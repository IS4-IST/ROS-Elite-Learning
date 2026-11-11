#include "rpc_client.h"
#include "net.h"

namespace ec {
namespace {

class ControllerCategory : public std::error_category {
public:
    const char* name() const noexcept override { return "ec-controller"; }
    std::string message(int code) const override { return "controller error " + std::to_string(code); }
};

// The controller sends results as JSON text, sometimes encoded twice ("\"V3.11.2\"")
nlohmann::json decode_result(nlohmann::json value) {
    for (int i = 0; i < 3 && value.is_string(); ++i) {
        auto parsed = nlohmann::json::parse(value.get<std::string>(), nullptr, /*allow_exceptions=*/false);
        if (parsed.is_discarded()) break;   // a plain string, not JSON text
        value = std::move(parsed);
    }
    return value;
}

}  // namespace

const std::error_category& controller_category() {
    static ControllerCategory category;
    return category;
}

RpcClient::RpcClient(asio::io_context& context, std::string ip, std::uint16_t port, std::chrono::milliseconds timeout)
    : context_(context), ip_(std::move(ip)), port_(port), timeout_(timeout), socket_(context) {}

RpcClient::~RpcClient() {
    std::error_code ignored;
    socket_.close(ignored);   // directly: we're on the context's thread, or it has stopped
}

// --- connection -------------------------------------------------------------------------
asio::awaitable<std::error_code> RpcClient::connect() {
    if (std::error_code ec = co_await connect_with_timeout(socket_, ip_, port_, timeout_)) co_return ec;
    connected_ = true;
    asio::co_spawn(context_, reader(), asio::detached);    // one reader for the connection's life
    co_return std::error_code{};
}

void RpcClient::close() {
    asio::post(context_, [this] {
        std::error_code ignored;
        socket_.shutdown(asio::ip::tcp::socket::shutdown_both, ignored);
        socket_.close(ignored);   // the reader's pending read ends: it fails the waiting calls
        connected_ = false;
    });
}

void RpcClient::fail_all(std::error_code ec) {
    for (auto& [id, p] : pending_) {
        p->done = true;
        p->reply.ec = ec;
        p->timer.cancel();        // wakes the waiting call
    }
}

// --- writing: one queue, one writer --------------------------------------------------------
std::uint64_t RpcClient::send(const std::string& method, const nlohmann::json& params) {
    const std::uint64_t id = ++next_id_;
    const nlohmann::json msg = {{"jsonrpc", "2.0"}, {"method", method}, {"params", params}, {"id", id}};
    outbox_.push_back(msg.dump() + "\n");
    if (!writing_) {
        writing_ = true;
        asio::co_spawn(context_, writer(), asio::detached);
    }
    return id;
}

asio::awaitable<void> RpcClient::writer() {
    while (!outbox_.empty()) {
        std::error_code ec;
        co_await asio::async_write(socket_, asio::buffer(outbox_.front()),
                                   asio::redirect_error(asio::use_awaitable, ec));
        if (ec) {                 // connection gone: the reader notices and fails the waiting calls
            outbox_.clear();
            writing_ = false;
            close();
            co_return;
        }
        outbox_.pop_front();      // only now: the buffer had to live during the write
    }
    writing_ = false;
}

void RpcClient::notify(std::string method, nlohmann::json params) {
    if (connected_) send(method, params);
}

// --- reading: one reader hands each reply to its call by id ----------------------------------
asio::awaitable<void> RpcClient::reader() {
    asio::streambuf buffer;
    for (;;) {
        std::error_code ec;
        const std::size_t n = co_await asio::async_read_until(socket_, buffer, '\n',
                                                              asio::redirect_error(asio::use_awaitable, ec));
        if (ec) {
            connected_ = false;
            fail_all(asio::error::not_connected);
            co_return;
        }
        const std::string line(asio::buffers_begin(buffer.data()), asio::buffers_begin(buffer.data()) + n);
        buffer.consume(n);

        const auto msg = nlohmann::json::parse(line, nullptr, /*allow_exceptions=*/false);
        if (msg.is_discarded() || !msg.contains("id") || !msg["id"].is_number_unsigned()) continue;
        const auto it = pending_.find(msg["id"].get<std::uint64_t>());
        if (it == pending_.end()) continue;   // a late reply after its call timed out: drop it
        Pending& p = *it->second;
        if (const auto err = msg.find("error"); err != msg.end()) {
            const int code = err->is_object() ? err->value("code", -1) : -1;
            p.reply.ec = std::error_code(code, controller_category());
            p.reply.value = *err;                 // keeps the controller's message text
        } else {
            p.reply.value = decode_result(msg.value("result", nlohmann::json()));
        }
        p.done = true;
        p.timer.cancel();                         // wakes the call
    }
}

// --- the call -----------------------------------------------------------------------------------
asio::awaitable<Reply> RpcClient::call(std::string method, nlohmann::json params,
                                       std::optional<std::chrono::milliseconds> timeout) {
    if (!connected_) co_return Reply{asio::error::not_connected, {}};
    Pending p(context_);
    p.timer.expires_after(timeout.value_or(timeout_));
    const std::uint64_t id = send(method, params);
    pending_[id] = &p;            // before we suspend: the reply can't be read earlier
    std::error_code ignored;
    co_await p.timer.async_wait(asio::redirect_error(asio::use_awaitable, ignored));
    pending_.erase(id);           // before the frame (and p) goes away
    if (!p.done) co_return Reply{asio::error::timed_out, {}};
    co_return std::move(p.reply);
}

}  // namespace ec
