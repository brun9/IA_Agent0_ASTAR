# IA_Agent0_ASTAR
Agent0 - Environment &amp; Agent - ASTAR Algorithm

*************************************************************************************************************************************************************************** 

                                         Artificial Inteligence - Search Algorithms - A* (A-STAR) - Universidade dos Açores 2020/2021

**********************************************************************     Base code    ******************************************************************************* 

  Agent0_ver2 
  
***************************************************************************     Added Code   ************************************************************************************ 

  - example_agent_search_run_astar.py - This file contains the Client side of the software, which when connected to main.py (Server) will allow the A* algorithm to start calculating the best path towards the goal. Here you can find the A* algorithm itself, the heuristics formula utilized (Manhattan city block), and the mark_"" functions which were added for better visualization of the different steps of execution, such as node mapping and path calculation.
  
  - config.json - Json file used to configure the connection, and map (environment) details such as weights, goal_coordinates, object_map, weight_map.
  
**************************************************************************    Project Goals   ***********************************************************************************

O objetivo deste projeto foi o de implementar o algoritmo de pesquisa A* sobre o código base do Agent0, usando a distância de Manhattan como heurística. Foram desenvolvidas duas diferentes instâncias – uma em que o agente conhece todos os obstáculos presentes no ambiente, outra em que são colocados obstáculos “invisíveis” – na segunda, ao encontrar o obstáculo, o agente pára e re-mapea o ambiente, voltando a precorrer, desta vez, o caminho re-calculado.

*****************************************************************************   Notes   ****************************************************************************************

Para o projeto, foi utilizado o Agent0_ver2, o qual foi fornecido pelo professor. Este programa consiste num “mapa” criado no código (Python) e num “agente” que possui, à priori, diversas funcionalidades de interação com o ambiente. Toda a interação é feita num ambiente Cliente/Servidor. 
O Agent0 já nos foi providenciado com a estutura base, preparada para que fossem criados e testados diversos algoritmos de pesquisa, como por exemplo a “Pesquisa em Largura” e a “Pesquisa em Profundidade”, experiências as quais nos ajudaram a melhor compreender e interpretar as diversas vertentes do complexo tema que é a Inteligência artificial. 
Finalmente, e de acordo com os requerimentos do projeto, foi implementado o algoritmo de pesquisa A*.

***************************************************************************   Programmers   *************************************************************************************
  
|    André Sousa    |    Bruno Viveiros    |    Gonçalo Almeida    |

********************************************************************************************************************************************************************************* 
