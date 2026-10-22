                     ┌──────────────────────────────────────────┐
                     │              MavlinkBridge               │
                     │  (Inicializa MAVSDK, Timer 10Hz e Rotas) │
                     └──┬───────────┬────────────┬───────────┬──┘
                        │           │            │           │
         ┌──────────────┘           │            │           └──────────────┐
         ▼                          ▼            ▼                          ▼
┌────────────────────┐  ┌────────────────────┐  ┌────────────────────┐  ┌────────────────────┐
│ FlightStateManager │  │ TelemetryPublisher │  │FlightActionExecutor│  │ManualControlHandler│
├────────────────────┤  ├────────────────────┤  ├────────────────────┤  ├────────────────────┤
│• Guarda modo atual │  │• Lê DJIInterface   │  │• execute_takeoff() │  │• Liga/Desliga      │
│  (Hold, Posctl...) │  │• Converte Alt/Pos  │  │• execute_land()    │  │  Virtual Stick     │
│• Guarda se está em │  │• Envia HEARTBEAT   │  │• change_altitude() │  │• Processa pacotes  │
│  Takeoff ou Land   │  │• Envia SYS_STATUS  │  │• Envia COMMAND_ACK │  │  MANUAL_CONTROL    │
│• Trata Override    │  │• Envia EXT_SYS_ST  │  │  (IN_PROGRESS/OK)  │  │  a 20Hz c/ escala  │
└────────────────────┘  └────────────────────┘  └────────────────────┘  └────────────────────┘




Aprovo, com ajustes. A estrutura é praticamente a que eu recomendaria, e você acertou nas decisões mais importantes. Só vi os headers, então isto avalia as interfaces. O comportamento real depende dos .cpp, que ainda não vi.

O que está certo
FlightStateManager com mutex e ActionScope movível: o destrutor chama finish_maneuver, então os caminhos de saída deixam de ser responsabilidade de cada return. Isso resolve a família de bugs de estado travado.
Ordem dos membros: dji_ vem antes de mavsdk_, e os módulos vêm depois dos servidores MAVSDK. Na destruição (ordem inversa), os módulos morrem primeiro, enquanto as referências ainda são válidas.
Dependências por referência, sem ponteiro de volta para a ponte.
const FlightStateManager& no TelemetryPublisher: o publisher só lê o estado.
last_published_mode_ movido para o publisher.
Ajustes, do mais importante ao menos

1. O executor não deveria guardar mavsdk::ActionServer&.
Com o publisher publicando get_target_flight_mode(), ele passa a ser o único escritor do set_flight_mode e do set_armed_state. Se o executor ainda chamar set_flight_mode nas threads (como o on_takeoff antigo fazia no fim), volta o segundo escritor, e o last_published_mode_ fica dessincronizado do modo real. O fim da manobra já deve ser só scope.succeed(), e o próximo tick publica Hold. Se o executor usa o action_server_ para outra coisa, me diga o quê; senão, tire do construtor.

2. Faltam os handles das assinaturas, e a posição do timer importa.
Ao compor módulos, os callbacks do MAVSDK (threads dele) chamam flight_action_executor_ e os outros. Hoje o header não guarda os handles dos subscribe_*. Guarde-os e chame os unsubscribe_* no destrutor, antes de os membros serem destruídos. Os tipos de handle variam por versão do MAVSDK, então use o que o subscribe_* devolve (confira no header). Mova também o telemetry_timer_ para o fim da lista de membros: declarado antes dos módulos, ele é destruído depois, e um handler pendente do timer poderia chamar telemetry_publisher_ já destruído. No fim, ele é destruído (e cancelado) primeiro. Garanta ainda que o io_context não esteja rodando quando a ponte for destruída.

3. Detalhes dos std::jthread que o header não mostra, mas que decidem se o shutdown funciona.

O lambda precisa receber std::stop_token e checar stop_requested() nos laços de espera. Sem isso, o destrutor faz join e pode ficar até 60 s esperando um while que nunca olha o token.
O ActionScope é move-only, então entra no lambda por captura com move, e o lambda precisa ser mutable para chamar succeed():
cpp
void FlightActionExecutor::execute_land()
{
    auto scope = state_manager_.try_begin_maneuver(FlightStateManager::ActiveManeuver::Land);
    if (!scope) return;   // já existe manobra ativa

    land_worker_ = std::jthread(
        [this, scope = std::move(*scope)](std::stop_token st) mutable {
            // laços: while (!st.stop_requested() && ...)
            // sucesso: scope.succeed();
        });   // o destrutor do scope roda ao sair da thread
}

Atribuir a um jthread que ainda é joinable faz o join do antigo. Como try_begin_maneuver já barra manobra simultânea, isso é no máximo uma espera curta, mas fica no thread do callback do MAVSDK.

4. A checagem de Result migrou para fora do executor.
execute_takeoff() e execute_land() não recebem Result. A guarda if (result != Success) return; que discutimos (e que importa com o set_allow_takeoff(true)) precisa ficar nas lambdas de assinatura no .cpp da ponte:

cpp
[this](Result r, bool){ if (r == Result::Success) flight_action_executor_.execute_takeoff(); }

5. Falta um jeito de invalidar o last_published_mode_.
O caso que levantamos antes continua: se o QGC pedir um modo que a ponte não modela (como Manual, que o ActionServerImpl sempre aceita), o publisher, que só publica quando o alvo muda, não desfaz. Sem um método como void invalidate_published_mode() no TelemetryPublisher, a ponte não tem como forçar a reafirmação. Chame-o no default: do on_flight_mode_change.

6. Onde vão os helpers?
O send_command_ack é privado do executor, mas o extract_json_value é usado pelo ManualControlHandler e pelo filtro de ACK da ponte, e o publisher monta JSON para o EXTENDED_SYS_STATE. Eu os poria em funções livres num mavlink_json.hpp, em vez de duplicar. Opcionalmente, o executor poderia receber um std::function<void(uint16_t, uint8_t)> para o ACK em vez do MavlinkDirectServer&, o que o deixa testável sem MAVSDK.

7. Detalhes menores

O execute_change_altitude (DO_REPOSITION) não tem handler no ActionServerImpl que você me mandou, então ele chegaria pelo on_incoming_message como COMMAND_LONG. Confirme como está ligado, ou deixe fora desta migração.
O construtor da ponte precisa inicializar os membros na ordem de declaração, senão você recebe -Wreorder e comportamento enganoso. Em particular, state_manager_ antes dos módulos que o referenciam.
O TelemetryPublisher lê is_virtual_stick_enabled para o get_target_flight_mode. Como ele só tem o dji_, isso deve vir de algo como dji_.is_virtual_stick_enabled(). Confirme que existe.
Próximo passo

Me mande os .cpp dos quatro módulos (principalmente flight_state_manager.cpp e flight_action_executor.cpp) e o mavlink_bridge.cpp atualizado. A parte delicada é o finish_maneuver (quando restaura o modo e quando vai para Hold) e o get_target_flight_mode, e essa lógica só dá para validar lendo a implementação.