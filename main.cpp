#include <iostream>
#include "grafo.h"

using namespace std;

int main()
{
    Grafo<char> grafo1(NO_DIRIGIDO);
    grafo1.insertar_vertice('A');
    grafo1.insertar_vertice('B');
    grafo1.insertar_vertice('C');
    grafo1.insertar_vertice('D');
    grafo1.insertar_vertice('E');
    grafo1.insertar_arista('A', 'C', 8);
    grafo1.insertar_arista('A', 'B', 4);
    grafo1.insertar_arista('B', 'A', 4);
    grafo1.insertar_arista('B', 'C', 1);
    grafo1.insertar_arista('B', 'D', 2);
    grafo1.insertar_arista('C', 'A', 8);
    grafo1.insertar_arista('C', 'D', 4);
    grafo1.insertar_arista('C', 'E', 2);
    grafo1.insertar_arista('D', 'B', 2);
    grafo1.insertar_arista('D', 'C', 4);
    grafo1.insertar_arista('D', 'E', 7);
    grafo1.insertar_arista('E', 'C', 2);
    grafo1.insertar_arista('E', 'D', 7);
    grafo1.lista_adyacencia();
    cout << endl << "PRIM" << endl;
    grafo1.prim();
    cout << endl << "KRUSKAL" << endl;
    grafo1.kruskal();
    cout << endl << "FLOYD" << endl;
    grafo1.floyd();
    cout << endl << "DIJKSTRA CON A" << endl;
    grafo1.dijkstra('A');   
 /*
*/
    return 0;
}