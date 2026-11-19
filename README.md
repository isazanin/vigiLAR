![Banner vigiLAR](imagens/logo_lateral.png)

# vigiLAR — Robô Autônomo para Monitoramento e Segurança Residencial

**Projeto Acadêmico | Turma CDC.6NA**

## Sobre o Projeto

O **vigiLAR** é um projeto de desenvolvimento de um robô autônomo voltado para o monitoramento e a segurança residencial durante o período noturno.

A proposta consiste em desenvolver um robô capaz de realizar patrulhas pelo ambiente de forma independente, observando possíveis movimentações e identificando situações incomuns ou suspeitas.

Ao detectar uma possível ocorrência, o sistema deverá reagir ao evento e enviar um alerta ao responsável, permitindo que ele acompanhe a situação e tome as medidas necessárias.

Além do monitoramento, o robô deverá ser capaz de reconhecer o ambiente ao seu redor, navegar de maneira autônoma e evitar obstáculos durante sua movimentação.

## Objetivo do Projeto

Desenvolver uma solução inteligente de segurança residencial capaz de atuar durante o período noturno, realizando o monitoramento autônomo da residência e auxiliando na identificação de possíveis situações suspeitas, sem depender de controle manual constante.

## Tecnologias do Projeto

### Blender: 
Responsável pelo desenvolvimento da modelagem tridimensional do robô e do ambiente residencial, criando uma representação virtual adequada para a simulação.

### Unity:
Utilizado para integrar os modelos tridimensionais e desenvolver o ambiente de simulação, permitindo representar o funcionamento e a movimentação do robô pela residência.

### Pygame
Responsável pelo desenvolvimento do simulador 2D do robô vigiLAR, permitindo testar sua movimentação e navegação pelo ambiente residencial.

O simulador inclui recursos de planejamento de rotas com A*, sensor LiDAR simulado, desvio de obstáculos, patrulhamento autônomo, modo noturno e detecção simulada de movimentações suspeitas.

Os arquivos do simulador, suas dependências e instruções de execução estão disponíveis na pasta [pygame/](pygame/).

## Funcionalidades previstas

- Navegação autônoma pelo ambiente residencial.
- Realização de patrulhas noturnas independentes.
- Identificação do espaço ao redor do robô.
- Detecção e desvio de obstáculos.
- Monitoramento de movimentações.
- Identificação de possíveis situações suspeitas.
- Reação à detecção de ocorrências.
- Envio de alertas ao responsável.
- Simulação do funcionamento do robô em ambiente virtual.

## Etapas de Desenvolvimento

O desenvolvimento do projeto está organizado nas seguintes etapas:

1. Definição da proposta e dos requisitos do projeto.
2. Modelagem tridimensional do robô no Blender.
3. Modelagem do ambiente residencial.
4. Integração dos modelos ao Unity.
5. Desenvolvimento do ambiente de simulação.
6. Integração com Pygame.
7. Implementação da navegação autônoma e do desvio de obstáculos.
8. Desenvolvimento das funcionalidades de monitoramento e segurança.
9. Realização de testes e validação do sistema.

## Organização do Repositório

O repositório do **vigiLAR** está organizado nas seguintes pastas:

- **[Blender](blender/):** modelos 3D do robô, da casa e da planta baixa.
- **[Unity](unity/):** projeto de simulação tridimensional com a residência e o robô.
- **[Pygame](pygame/):** simulador 2D, código Python, planta baixa e instruções de execução.
- **[Imagens](imagens/):** identidade visual e logos do projeto.
- **[Documentos](documentos/):** documentação acadêmica do vigiLAR.
- **[Referências](referencias/):** materiais utilizados como referência durante o desenvolvimento.

## Vídeo Pitch — vigiLAR

Conheça o projeto **vigiLAR — Robô Autônomo para Monitoramento e Segurança Residencial** por meio do nosso vídeo pitch.

O vídeo apresenta a proposta acadêmica e uma visão geral do projeto.

**[Clique aqui para assistir ao vídeo pitch no YouTube](https://youtu.be/R9P-C9x_mQI)**

## Status do Projeto

**Projeto concluído — versão acadêmica.**

O **vigiLAR** é um projeto acadêmico de robótica e segurança residencial, com modelagem tridimensional e simulações funcionais.

O projeto conta com:

- Modelagem 3D do robô e da residência no Blender.
- Simulação tridimensional funcional no Unity, com movimentação do robô pelo ambiente residencial.
- Simulador 2D funcional desenvolvido em Python com Pygame.
- Recursos simulados de navegação autônoma, LiDAR, patrulhamento e monitoramento residencial no Pygame.
- Documentação acadêmica e arquivos disponibilizados no GitHub.

O projeto encontra-se concluído para fins de apresentação e avaliação acadêmica.

## Integrantes

**Turma: CDC.6NA**

- Isabella Santos de Albuquerque Zanin — 105446
- Carolina Vitória Celso Nunes — 106749
- Pedro Henrique Pedroso — 106627

## Considerações Finais

O projeto vigiLAR busca explorar a integração entre modelagem tridimensional, simulação e tecnologias de robótica para representar uma solução de segurança residencial inteligente.

Por meio da utilização do Blender, Unity e pygame, pretende-se desenvolver um ambiente virtual no qual seja possível simular o comportamento de um robô autônomo, sua navegação e suas funcionalidades de monitoramento e segurança.
