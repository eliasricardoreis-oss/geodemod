# A* Pilot (Geode mod, GD 2.2081)

Autopilot que simula a física do jogo e busca (A*) a melhor sequência de inputs.

## Estrutura
- `src/core/Core.hpp`    – mundo (blocos, spikes, portais, orbs, pads) + simulador por modo (cube, ship, ball, ufo, wave, robot, spider, swing)
- `src/core/Planner.hpp` – A* incremental. Classic: DFS com memória + custo de cliques/folga. Platformer: g = tempo, A* acha a rota mais rápida
- `src/core/Pilot.hpp`   – controlador: pré-planeja a fase, detecta drift sim×real, replaneja, pipeline, **memória de mortes** (menos tentativas)
- `src/main.cpp`         – ponte com o Geode (hooks, leitura de objetos/estado, inputs, desenho do caminho)
- `tests/test_core.cpp`  – testes sem o jogo: `g++ -std=c++17 -O2 -Isrc/core tests/test_core.cpp -o t && ./t`

## Build
Como no template-mods: `geode build` (precisa do Geode SDK/CLI) ou o workflow `.github/workflows/build.yml`.

## Calibração (importante)
As constantes em `Config` (gravidade, pulos, ship, orbs, pads...) são aproximações. Ative
"Log sim-vs-real drift" nas configs, jogue e compare `real y` × `pred y` no log do Geode;
ajuste `Config` até o drift ficar baixo. O mod replaneja sozinho quando o drift passa de `driftTol`.

## Limitações conhecidas
- Sem dual mode, mirror portals, dash orbs, spider orbs/pads, slopes (viram caixas), triggers de movimento/rotação, objetos móveis.
- Pontos marcados `VERIFY` em `main.cpp` (nomes de campos/enums, IDs, escala de velocidades) podem precisar de ajuste nos headers do Geode.
- Platformer: a meta é `m_levelLength`; ajuste `goalX` se sua fase terminar de outro jeito.
- Não garante completar qualquer fase; é um planejador, não um TAS perfeito.
