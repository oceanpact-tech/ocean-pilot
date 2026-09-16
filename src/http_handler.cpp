#include "http_handler.hpp"


// Método principal de envio assíncrono (Fire-and-Forget)
void HttpHandler::post_async(
        const std::string& endpoint, 
        const std::string& payload,
        std::function<void()> on_complete) 
{  
    // NOTA: Passamos as variáveis por VALOR [host, port, endpoint, payload]
    // para a thread, garantindo que não teremos falha de segmentação 
    // caso as variáveis originais sejam destruídas antes da thread terminar.
    
    std::thread([host = host_, port = port_, endpoint, payload, on_complete]() {
        try {
            net::io_context ioc;
            tcp::resolver resolver(ioc);
            beast::tcp_stream stream(ioc);

            // Resolve DNS/IP e conecta
            auto const results = resolver.resolve(host, port);
            stream.connect(results);

            // Prepara a requisição HTTP POST
            http::request<http::string_body> req{http::verb::post, endpoint, 11};
            req.set(http::field::host, host);
            req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
            req.set(http::field::content_type, "text/plain");
            req.body() = payload;
            req.prepare_payload();

            // Envia
            http::write(stream, req);

            // Lê a resposta (opcional, útil para debug)
            beast::flat_buffer buffer;
            http::response<http::string_body> res;
            beast::error_code ec;
            http::read(stream, buffer, res, ec);
            
            if (ec) {
                // Se o servidor for "preguiçoso" e mandar HTTP mal formatado,
                // apenas registramos o aviso ao invés de tratar como erro fatal.
                std::cout << "[Aviso] Resposta fora do padrao no endpoint [" << endpoint << "]: " << ec.message() << "\n";
                if (!res.body().empty()) {
                    std::cout << "Corpo lido (parcial): " << res.body() << "\n";
                }
            } else {
                std::cout << "Resposta do drone [" << endpoint << "]: " << res.body() << "\n";
            }
            
            // Encerra a conexão de forma limpa
            stream.socket().shutdown(tcp::socket::shutdown_both, ec);

            // Avisa quem chamou que o trabalho terminou
            if (on_complete) {
                on_complete();
            }
        } 
        catch (std::exception const& e) {
            std::cerr << "[HttpHandler] Falha no POST para " << endpoint << ": " << e.what() << '\n';
        
            // Chama o callback mesmo com erro para não travar o programa principal
            if (on_complete) {
                on_complete();
            }
        }
    }).detach(); // .detach() libera a thread para rodar solta em background
}
