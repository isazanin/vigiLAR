# vigiLAR - CDC.6NA — robô móvel inteligente de vigilância residencial (simulador pygame, IMR)

Rodar:

    pip install -r requirements.txt
    python vigilar.py
(deixe a imagem `Planta_Baixa_Modern_House.jpeg` na mesma pasta)

Testar (sem abrir janela):

    python testes_vigilar.py

## Teclas
Clique esquerdo: ir até o ponto | 1-6: Cozinha, Jantar, Sala, Quarto, Banheiro, Entrada
P: patrulha | E: explorar sozinho (sem usar o mapa pronto) | TAB: manual (setas) | S: parar
**T: modo NOTURNO (inicia a ronda e liga os alertas)** | **I: intruso de teste (pessoa simulada)**
Botão direito: coloca/remove caixa surpresa | V: planta / mapa aprendido / grade inflada
L: lidar | B: barreiras | O: odometria | U: guiar pela odometria | Z: relocalizar
N: ruído | ESPAÇO: pausa | [ ]: velocidade | R: reiniciar

## Vigilância (módulos novos)
- **Ronda autônoma (P):** o robô visita os cômodos que consegue alcançar, planejando com A* na planta
  e desviando de obstáculos pelo LiDAR (FSM da V1).
- **Modo noturno (T):** ronda mais lenta (`V_NOITE`), visão escurecida na tela e alertas ligados.
- **Detecção de movimentação suspeita (`SecurityNode`):** compara o scan MEDIDO com o scan ESPERADO da planta
  (só paredes e móveis). Raios mais curtos que o esperado formam alvos; cada alvo é acompanhado.
  Parado = "objeto novo" (INFO). Em movimento (> 0,30 m em 1,5 s) = pessoa; à noite gera **ALERTA CRÍTICO**.
  De dia, o movimento é só registrado (sem alerta).
- **Envio de alertas:** publicados no tópico `/alerta`, mostrados no painel da tela e gravados
  em `alertas_vigilar.log` (um JSON por linha). Opcional: defina a variável de ambiente
  `VIGILAR_WEBHOOK` com uma URL para também receber cada alerta por HTTP POST (JSON).
  `VIGILAR_LOG` troca o caminho do arquivo de log.

## Planta
A planta foi corrigida a partir das marcações vermelhas (`referencias/`): paredes laterais, inferior e superior
agora têm a espessura real, a frente do guarda-roupa do quarto e as paredes inclinadas pela perspectiva viraram
polígonos, e a divisória central é contínua. **A planta não tem aberturas entre os três ambientes**
(sala/cozinha, quarto, banheiro), então a ronda cobre Cozinha, Jantar, Sala e Entrada; Quarto e Banheiro
ficam isolados e o robô responde "SEM_ROTA" em vez de atravessar a parede. Para testar com portas,
edite `PASSAGENS` e coloque `PASSAGENS_ATIVAS = True` no topo de `vigilar.py`.

Ajustar o mapa: listas PAREDES, PAREDES_POLI, MOVEIS, PASSAGENS e COMODOS no topo de `vigilar.py` (tecla B mostra as barreiras).
