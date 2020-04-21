#include <iostream>
#include "graph.h"

using namespace std;

int main()
{
    Grafo<char> grafo1(NO_DIRIGIDO);
    grafo1.insertar_vertice('A');
    grafo1.insertar_vertice('B');
    grafo1.insertar_vertice('C');
    grafo1.insertar_vertice('D');
    grafo1.insertar_vertice('E');
    /*
    grafo1.insertar_vertice('F');
    grafo1.insertar_vertice('G');
    grafo1.insertar_vertice('H');
    */
    
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
    cout << endl << "PRIM" << endl;
    grafo1.prim();
    cout << endl << "KRUSKAL" << endl;
    grafo1.kruskal();
    cout << endl << "FLOYD" << endl;
    grafo1.floyd();
    cout << endl << "DIJKSTRA CON A" << endl;
    grafo1.dijkstra('A');   


    /*
    Grafo<int> g1;    
    for(int i(0); i<10; ++i){
        g1.insertar_vertice(i);
    }
    for (int i(0); i < 10; ++i){
        g1.insertar_arista(g1.getVertice(5), g1.getVertice(i), (i*2));
    }
    g1.insertar_vertice(5);
    g1.insertar_arista(g1.getVertice(5), g1.getVertice(1), 100);

    g1.eliminarArista(g1.getVertice(5), g1.getVertice(5));
    g1.eliminarArista(g1.getVertice(5), g1.getVertice(0));
    //g1.eliminarVertice(g1.getVertice(5));

    cout << "Tamano: " << g1.tamano() << endl;
    g1.listaAdyacencia();
    cout << "Se hace un clear()" << endl;
    g1.limpiar();
    cout << "Tamano: " << g1.tamano();
    */
    return 0;
}