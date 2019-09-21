# LoRaMESHesp32

Biblioteca do módulo LoRaMESH da radioenge adaptada para ESP32.

Esta biblioteca implementa apenas algumas funções disponíveis para a Serial de Comando do módulo
LoRaMESH utilizando a Serial1 da ESP32 na GPIO16(rx) e GPIO17(tx).

O exemplo (example_rssi_req.ino) implementa uma requisição de RSSI do nó mestre para um nós escravo
com id SlaveID e decodifica a resposta recebida mostrando o valor do RSSI capturado na ida e na volta
do sinal.

Placa testada:
EspDevkit - V4
VisualStudioCode com PlatformIO
Framework: Arduino


