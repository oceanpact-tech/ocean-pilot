#include "dji_interface.hpp"

/* @brief Starts telemetry stream
 */
// DJIInterface::DJIInterface(const std::string& rc_ip) {
//     // TO DO: fazer verificação de auto descovery se ip for uma string vazia
// }

/* @brief Stops telemetry stream
 */
DJIInterface::~DJIInterface() {
    // this->stop_telemetry_stream();
}


void DJIInterface::request_takeoff(std::function<void()> on_complete) {
    // Converte a string_view do namespace para std::string e envia vazio no payload
    this->http_client_.post_async(std::string(endpoints::TAKEOFF), "", on_complete);
}