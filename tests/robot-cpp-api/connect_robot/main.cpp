// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.
#include <Elite/Log.hpp>
#include <Elite/PrimaryPortInterface.hpp>
#include <boost/asio.hpp>
#include <boost/program_options.hpp>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

using boost::asio::ip::tcp;
using namespace std::chrono;
using namespace ELITE;
namespace po = boost::program_options;

#define SERVER_PORT 50002

// The robot will send string
#define ROBOT_SOCKET_SEND_STRING "hello"

// The TCP server class
class TcpServer {
   public:
    TcpServer(boost::asio::io_context& io_context, short port)
        : io_context_(io_context), acceptor_(io_context, tcp::endpoint(tcp::v4(), port)) {
        startAccept();
    }

    std::future<std::string> getReceiveString() {
        std::lock_guard<std::mutex> lock(promise_mutex_);
        std::promise<std::string> promise;
        auto future = promise.get_future();
        promise_queue_.push(std::move(promise));
        return future;
    }

   private:
    void startAccept() {
        auto new_socket = std::make_shared<tcp::socket>(io_context_);
        acceptor_.async_accept(*new_socket, [this, new_socket](const boost::system::error_code& error) {
            if (!error) {
                std::cout << "New connection from: " << new_socket->remote_endpoint().address().to_string() << std::endl;
                startRead(new_socket);
            }
            startAccept();  // Continue accepting next client
        });
    }

    void startRead(std::shared_ptr<tcp::socket> socket) {
        boost::asio::async_read_until(*socket, receive_buffer_, '\n',
                                      [this, socket](const boost::system::error_code& ec, size_t bytes_transferred) {
                                          if (!ec) {
                                              std::istream is(&receive_buffer_);
                                              std::string message;
                                              std::getline(is, message);

                                              std::lock_guard<std::mutex> lock(promise_mutex_);
                                              if (!promise_queue_.empty()) {
                                                  auto promise = std::move(promise_queue_.front());
                                                  promise_queue_.pop();
                                                  promise.set_value(message);
                                              } else {
                                                  std::cerr << "Warning: Received message but no waiting future.\n";
                                              }

                                              // Continue reading next message
                                              startRead(socket);
                                          } else {
                                              std::cerr << "Read error: " << ec.message() << std::endl;
                                              socket->close();
                                          }
                                      });
    }

    boost::asio::io_context& io_context_;
    tcp::acceptor acceptor_;
    boost::asio::streambuf receive_buffer_;

    std::mutex promise_mutex_;
    std::queue<std::promise<std::string>> promise_queue_;
};

int main(int argc, char** argv) {
    std::string robot_ip = "";
    std::string local_ip = "";

    // Parser param
    po::options_description desc(
        "Usage:\n"
        "\t./connect_robot_test <--robot-ip=ip> [--local-ip=\"\"]\n"
        "Parameters:");
    desc.add_options()
        ("help,h", "Print help message")
        ("robot-ip", po::value<std::string>(&robot_ip)->required(),
            "\tRequired. IP address of the robot.")
        ("local-ip", po::value<std::string>(&local_ip)->default_value(""), 
            "\tOptional. IP address of the local network interface.");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }

        po::notify(vm);
    } catch (const po::error& e) {
        std::cerr << "Argument error: " << e.what() << "\n\n";
        std::cerr << desc << "\n";
        return 1;
    }

    try {
        // The backend captures exception packets by parsing the packet data from the robot's primary port. Start a TCP server and
        // listen on a background thread.
        boost::asio::io_context io_context;
        TcpServer server(io_context, SERVER_PORT);
        std::thread thread([&io_context]() {
            try {
                io_context.run();
            } catch (const std::exception& e) {
                ELITE_LOG_FATAL("IO context error: %s", e.what());
            }
        });
        // Connect to robot primary port
        PrimaryPortInterface primary;
        primary.connect(robot_ip);
        if (local_ip.empty()) {
            local_ip = primary.getLocalIP();
        }
        // Construct script
        std::string robot_script = "def socket_test():\n";
        robot_script += "\tsocket_open(\"" + local_ip + "\", " + std::to_string(SERVER_PORT) + ")\n";
        robot_script += "\tsocket_send_string(\"" ROBOT_SOCKET_SEND_STRING "\\n\")\n";
        robot_script += "end\n";
        // Print script
        ELITE_LOG_INFO("Will send the script to robot primary port:\n %s", robot_script.c_str());
        // Send script to robot primary port
        primary.sendScript(robot_script);

        // Wait receive string and determine if it is correct
        auto future_recv = server.getReceiveString();
        if (future_recv.wait_for(5s) == std::future_status::ready) {
            auto str = future_recv.get();
            if (str == ROBOT_SOCKET_SEND_STRING) {
                ELITE_LOG_INFO("Success, robot connected to PC");
            } else {
                ELITE_LOG_ERROR("Fail, robot connected to PC but not send right string");
            }
        } else {
            ELITE_LOG_ERROR("Time out, robot not connect to PC");
        }

        io_context.stop();
        thread.join();

    } catch (std::exception& e) {
        ELITE_LOG_FATAL("Exception: %s", e.what());
    }

    return 0;
}