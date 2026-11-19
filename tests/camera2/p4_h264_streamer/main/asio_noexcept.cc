// Asio is built with ASIO_NO_EXCEPTIONS (C++ exceptions are off in ESP-IDF): it then calls
// asio::detail::throw_exception() for errors that have no error_code overload, and the
// application provides it. Those errors are unrecoverable here, so log and abort.
// main/CMakeLists.txt links this component WHOLE_ARCHIVE: only the asio library refers to
// these functions and it is linked after main, so the linker would otherwise drop them.
#include <cstdlib>
#include <new>
#include <system_error>

#include <utility>
#include <asio.hpp>

#include "esp_log.h"

namespace asio::detail {

template <typename Exception>
void throw_exception(const Exception& e) {
    ESP_LOGE("asio", "fatal: %s", e.what());
    abort();
}

template void throw_exception<std::system_error>(const std::system_error&);
template void throw_exception<std::bad_alloc>(const std::bad_alloc&);
template void throw_exception<asio::invalid_service_owner>(const asio::invalid_service_owner&);
template void throw_exception<asio::service_already_exists>(const asio::service_already_exists&);
template void throw_exception<asio::ip::bad_address_cast>(const asio::ip::bad_address_cast&);
template void throw_exception<asio::execution::bad_executor>(const asio::execution::bad_executor&);

}  // namespace asio::detail
