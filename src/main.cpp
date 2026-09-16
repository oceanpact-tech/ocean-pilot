#include "dji_interface.hpp"

#include <mutex>
#include <condition_variable>
#include <iostream>

int main(int argc, char* argv[]) {

    DJIInterface dji("192.168.228.24");

    // Variáveis para a sincronização
    std::mutex mtx;
    std::condition_variable cv;
    bool requisicao_concluida = false;

    std::cout << "Conectado! Enviando comando de teste..." << std::endl;

    // Dispara a requisição e passa uma função lambda como callback
    dji.request_takeoff([&]() {
        // Este bloco será executado DENTRO da thread do HTTP quando ela terminar
        std::lock_guard<std::mutex> lock(mtx);
        requisicao_concluida = true;
        cv.notify_one(); // Desperta a thread principal!
    });

    // --- Thread Principal ---
    // Cria um lock único e usa a condition_variable para dormir (sem consumir CPU)
    // O wait só libera quando a thread do HTTP chamar o notify_one() E o booleano for true
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [&]() { return requisicao_concluida; });

    std::cout << "O programa principal acordou! Resposta recebida. Encerrando." << std::endl;

    return 0;
}