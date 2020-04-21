#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <queue>
#define INF 1<<30
#define DIRIGIDO true
#define NO_DIRIGIDO false
using namespace std;

//// PROTOTIPOS ////
template<typename T>
class Vertice;

template<typename T>
class Arista;

template<typename T>
class Grafo;

template<typename T>
struct Nodo;

///////////////////////////////////////////////////
///////////////////// VERTICE /////////////////////
///////////////////////////////////////////////////
template<typename T>
class Vertice
{
    Vertice<T> *sig;
    Arista<T> *aristaAdy;
    T dato;
    friend class Grafo<T>;
public:
    Vertice(){}
    ~Vertice(){}
};

///////////////////////////////////////////////////
///////////////////// ARISTA //////////////////////
///////////////////////////////////////////////////
template<typename T>
class Arista
{
    double peso;
    Arista<T>* sig;
    Vertice<T> *verticeAdy;

    friend class Grafo<T>;
public:
    Arista(){}
    ~Arista(){}
};

///////////////////////////////////////////////////
///////////////////// GRAFO ///////////////////////
///////////////////////////////////////////////////
template<typename T>
class Grafo
{
    Vertice<T>* raiz;
    Vertice<T>* ultimo;
    Arista<T>* getAristaFinal(Vertice<T>& vert);
    bool dirigido;

    void insertar_arista_p(Vertice<T> *origen, Vertice<T> *destino, double peso);
    void eliminar_arista_p(Vertice<T>* origen, Vertice<T>* destino);
    long buscar_nodo(Nodo<T> arrNodo[], Vertice<T> *&vertice);
    long buscar_pos_vertice(T* arrVer[], Vertice<T>*& dato);
    void imprimir_mejor_camino(Nodo<T>arrNodo[]);
    void init_matriz_distancia(vector<vector<long>> &matriz, T *arrDatos[]);
public:
    Grafo(bool _dirigido = DIRIGIDO) : raiz(nullptr), dirigido(_dirigido) {}
    ~Grafo(){ limpiar(); }

    Vertice<T>* getVertice(T dato);
    void insertar_arista(T origen, T destino, double peso);
    bool vacio();
    int size();
    void insertar_vertice(T dato);
    void lista_adyacencia();
    void limpiar();
    void eliminar__arista(T origen, T destino);
    void eliminar_vertice(T vertice);
    void dijkstra(T origen);
    void floyd();
    void kruskal ();
    void prim();
};

template<typename T>
bool Grafo<T>::vacio()
{ return raiz == nullptr; }

template<typename T>
int Grafo<T>::size()
{
    Vertice<T> *aux;
    int cont = 0;
    
    aux = raiz;
    while (aux != nullptr){
        aux = aux->sig;
        ++cont;
    }
    return cont;
}

template<typename T>
Vertice<T>* Grafo<T>::getVertice(T dato)
{
    Vertice<T> *aux;
    aux = raiz;
    while (aux != nullptr){
        if (aux->dato == dato)
            return aux;
        
        aux = aux->sig;
    }
    return nullptr;
}

template<typename T>
void Grafo<T>::insertar_vertice(T data)
{
    Vertice<T> *nuevo = new Vertice<T>;
    nuevo->dato = data;
    nuevo->aristaAdy = nullptr;
    nuevo->sig = nullptr;
    if (vacio())
        raiz = nuevo;
    else if(getVertice(data) != nullptr){
        cout << "El elemento ya ha sido insertado" << endl; 
        delete nuevo;
    }
    else{
        Vertice<T>* aux;
        aux = raiz;
        while(aux->sig != nullptr)
            aux = aux->sig;

        aux->sig = nuevo;
    }
}

template <typename T>
void Grafo<T>::insertar_arista_p(Vertice<T> *origen, Vertice<T> *destino, double _peso)
{
    bool repetido = false;
    Arista<T> *nueva = new Arista<T>;
    Arista<T> *aux;
    Vertice<T>* vertAux;

    nueva->peso = _peso;
    nueva->sig = nullptr;
    nueva->verticeAdy = nullptr;
    aux = origen->aristaAdy;
    
    if(aux == nullptr){
        origen->aristaAdy = nueva;
        nueva->verticeAdy = destino;
    }
    else{
        while(aux->sig != nullptr){
            if (aux->verticeAdy == destino){
                cout << "La arista ya existe" << endl;
                repetido = true;
                delete nueva;
                break;
            }
            aux = aux->sig;
        }
        
        if (!repetido){
            aux->sig = nueva;
            nueva->verticeAdy = destino;
        }
    }
}

template <typename T>
void Grafo<T>::insertar_arista(T origen, T destino, double peso)
{
    Vertice<T>* origenTmp = getVertice(origen);
    Vertice<T> *destinoTmp = getVertice(destino);
    insertar_arista_p(origenTmp, destinoTmp, peso);

    if (dirigido == NO_DIRIGIDO)
        insertar_arista_p(destinoTmp, origenTmp, peso);
}

template<typename T>
void Grafo<T>::lista_adyacencia()
{
    Vertice<T> *vertAux;
    Arista<T> *arisAux;
    vertAux = raiz;
    while(vertAux != nullptr){
        arisAux = vertAux->aristaAdy;

        while (arisAux != nullptr){
            cout << vertAux->dato << "->" 
                 << arisAux->verticeAdy->dato
                 << endl;
            arisAux = arisAux->sig;
        }
        vertAux = vertAux->sig;
    }
}

template <typename T>
Arista<T> *Grafo<T>::getAristaFinal(Vertice<T> &vert)
{
    Arista<T>* aux;
    aux = vert->aristaAdy;
    if (aux == nullptr)
        return nullptr;
    while (aux->sig != nullptr)
        aux = aux->sig;    
    return aux;
}

template<typename T>
void Grafo<T>::limpiar()
{
    Vertice<T> *vertAux;
    Arista<T> *arisAux;
    while (raiz != nullptr){
        vertAux = raiz;
        while (vertAux->aristaAdy != nullptr){
            arisAux = vertAux->aristaAdy;
            vertAux->aristaAdy = vertAux->aristaAdy->sig;
            delete arisAux;
        }
        raiz = raiz->sig;
        delete vertAux;
    }
}

template <typename T>
void Grafo<T>::eliminar_arista_p(Vertice<T> *origen, Vertice<T> *destino)
{
    bool encontrado = false;
    Arista<T>* aristaActual;
    Arista<T>* aristaAnterior;
    aristaActual = origen->aristaAdy;
    
    if (aristaActual == nullptr)
        cout << "El vértice origen no tiene aristas" << endl;
    else if(aristaActual->verticeAdy == destino){
        origen->aristaAdy = aristaActual->sig;
        delete aristaActual;
    }
    else{
        while(aristaActual != nullptr){
            if (aristaActual->verticeAdy == destino){
                encontrado = true;
                aristaAnterior->sig = aristaActual->sig;
                delete aristaActual;
                break;
            }
            aristaAnterior = aristaActual;
            aristaActual = aristaActual->sig;
        }
        if(!encontrado)
            cout << "Los vértices no están conectados" << endl;
    }
}

template<typename T>
void Grafo<T>::eliminar__arista(T origen, T destino)
{
    Vertice<T>* origenTmp = getVertice(origen);
    Vertice<T> *destinoTmp = getVertice(destino);
    eliminar_arista_p(origenTmp, destinoTmp);
    if (dirigido == NO_DIRIGIDO)
        eliminar_arista_p(destinoTmp, origenTmp);
}

template<typename T>
void Grafo<T>::eliminar_vertice(T datoOrigen)
{
    Vertice<T> *vertice = getVertice(datoOrigen);
    Vertice<T> *vertActual;
    Vertice<T> *vertAnterior;
    Arista<T> *aristaAux;
    vertActual = raiz;
    // Elimina las conexiones entre el vértice a 
    // borrar y el resto de vértices
    while(vertActual != nullptr){
        aristaAux = vertActual->aristaAdy;
        while(aristaAux != nullptr){
            if (aristaAux->verticeAdy == vertice){
                eliminar__arista(vertActual->dato, aristaAux->verticeAdy->dato);
                break;
            }
            aristaAux = aristaAux->sig;
        }
        vertActual = vertActual->sig;
    }
    
    //El vertice a borrar es la raíz
    vertActual = raiz;
    if (raiz == vertice)
        raiz = raiz->sig;
    
    //Caso contrario
    else{
        while(vertActual != vertice){
            vertAnterior = vertActual;
            vertActual = vertActual->sig;
        }
        vertAnterior->sig = vertActual->sig;
    }

    //Se borran las aristas del vértice
    while (vertActual->aristaAdy != nullptr){
        aristaAux = vertActual->aristaAdy;
        vertActual->aristaAdy = vertActual->aristaAdy->sig;
        delete aristaAux;
    }
    //Se borra el vértice
    delete vertActual;
}

template<typename T>
struct cmp
{
    bool operator() (Nodo<T>*& a, Nodo<T>*&b)
    { return a->acumulado > b->acumulado; }
};

template<typename T>
struct Nodo
{
    Nodo(): actual(nullptr), anterior(nullptr),
            visitado(false), acumulado(INF){}
    Vertice<T>* actual;
    Vertice<T>* anterior;
    bool visitado;
    size_t acumulado;
};

template<typename T>
long Grafo<T>::buscar_nodo(Nodo<T> arrNodo[], Vertice<T>*& vertice)
{
    for (long i = 0; i < size(); ++i){
        if (arrNodo[i].actual->dato == vertice->dato)
            return i;
    }
    return -1;
}

template <typename T>
long Grafo<T>::buscar_pos_vertice(T* arrVer[], Vertice<T>*& dato)
{
    for (long i = 0; i < size(); ++i){
        if (*arrVer[i] == dato->dato)
            return i;
    }
    return -1;
}

template<typename T>
void Grafo<T>::imprimir_mejor_camino(Nodo<T>arrNodo[])
{
    long pos;
    Nodo<T>* nodoAdyacente;

    for (size_t i = 0; i < size(); ++i){
        cout << "peso acumulado: " << arrNodo[i].acumulado << endl;
        nodoAdyacente = &arrNodo[i];
        while (nodoAdyacente->anterior != nullptr){
            cout << nodoAdyacente->actual->dato << "->";
            pos = buscar_nodo(arrNodo, nodoAdyacente->anterior);
            if (pos == -1)
                break;
            nodoAdyacente = &arrNodo[pos];
        }
        cout << nodoAdyacente->actual->dato
             << endl << endl;
    }
}

template <typename T>
void Grafo<T>::dijkstra(T datoOrigen)
{
    Vertice<T>* origen = getVertice(datoOrigen);
    Nodo<T> arrNodo[size()];
    priority_queue<Nodo<T>*, vector<Nodo<T>*>, cmp<T>> cola;
    // Representa "u"
    Nodo<T> *nodoActual;
    // Representa "v"
    Nodo<T> *nodoAdyacente;
    Vertice<T>* vertAux;
    Arista<T>* arisAux;
    long pos;

    // Se inicializan todos los datos del array de nodos
    vertAux = raiz;
    for (size_t i = 0; i < size(); ++i){
        if (vertAux->dato == origen->dato){
            pos = long(i);
            arrNodo[i].acumulado = 0;
        }
        arrNodo[i].actual = vertAux;
        vertAux = vertAux->sig;
    }

    // Se añade el origen a la cola de prioridad
    cola.push(&arrNodo[pos]);
    // Se ejecuta mientras haya nodos sin visitar (nodos en la cola)
    while(!cola.empty()){
        // nodoActual guarda el dato mínimo de la cola 
        // (es decir, el dato mínimo que aún no haya sido visitado)
        nodoActual = cola.top();
        // Se borra el puntero de la cola y nos quedamos con el auxiliar
        cola.pop();
        // Lo "visitamos"
        nodoActual->visitado = true;
        // Guardamos una de las aristas adyacentes del vértice actual para
        // así poder visitar sus vértices adyacentes
        arisAux = nodoActual->actual->aristaAdy;
        // Se recorren todas las aristas adyacentes al vértice actual
        while(arisAux != nullptr){
            // Se obtiene el vértice adyacente a la arista actual
            vertAux = arisAux->verticeAdy;
            // Se obtiene la posición del nodo en arrNodo
            pos = buscar_nodo(arrNodo, vertAux);
            nodoAdyacente = &arrNodo[pos]; 
            if (!nodoAdyacente->visitado && nodoAdyacente != nullptr){
                if (nodoAdyacente->acumulado > nodoActual->acumulado + arisAux->peso){
                    nodoAdyacente->acumulado = nodoActual->acumulado + arisAux->peso;
                    nodoAdyacente->anterior = nodoActual->actual;
                    cola.push(nodoAdyacente);
                }
            }
            arisAux = arisAux->sig;
        }
    }
    imprimir_mejor_camino(arrNodo);
}

template <typename T>
void Grafo<T>::floyd()
{
    vector<vector<long>> matDistancias(size());
    Vertice<T>* matRecorrido[size()][size()];
    Vertice<T>* verAux = raiz;
    T* arrDatos[size()]; 
    long posAux;

    // Inicializa la matriz de recorrido
    for(long i = 0; i < size(); i++){
        for (long j = 0; j < size(); j++){
            matDistancias[i].push_back(INF);
            if (i != j)
                matRecorrido[j][i] = verAux;
            else{
                matDistancias[i][i] = 0;
                matRecorrido[j][i] = nullptr;
            }
        }
        arrDatos[i] = &verAux->dato;
        verAux = verAux->sig;
    }
    init_matriz_distancia(matDistancias, arrDatos);

    // Se ejecuta el algoritmo Floyd
    for(int k = 0; k < size(); k++){
        for(int i = 0; i < size(); i++)
            for(int j = 0; j < size(); j++){
                unsigned long dt = matDistancias[i][k] + matDistancias[k][j];
                if (matDistancias[i][j] > dt && j != k && i != k){
                    matDistancias[i][j] = dt;
                    matRecorrido[i][j] = getVertice(*arrDatos[k]);
                }
            }
    }
    // Se imprimen los recorridos de cada vértice
    for (int i = 0; i < size(); i++){
        for (int j = 0; j < size(); ++j){
            posAux = i;
            verAux = matRecorrido[posAux][j];
            cout << *arrDatos[i];
            while(verAux != nullptr){
                cout << "->" << verAux->dato;
                posAux = buscar_pos_vertice(arrDatos, verAux);
                verAux = matRecorrido[posAux][j];
            }
            cout << " (distancia: "
                 << matDistancias[i][j] << ")" << endl;
        }
        cout << endl;
    }
}

template <typename T>
void Grafo<T>::kruskal()
{
    vector<vector<long>> matAdyacencia(size());
    long matArbol[size()][size()];
    T* pertenece[size()];
    T *datos[size()];
    pair<T*, long> nodoA;
    pair<T*, long> nodoB;
    long min;
    Vertice<T>* verAux = raiz;

    if (dirigido == DIRIGIDO){
        cout << "No es posible realizar este algoritmo" << endl
             << "ya que se requiere de un grafo NO DIRIGIDO" << endl;
        return;
    }

    // Se inicializan ambas matrices
    for (long i = 0; i < size(); ++i){
        for (long j = 0; j < size(); ++j){
            matAdyacencia[i].push_back(INF);
            if (i == j)
                matAdyacencia[i][i] = 0;
            matArbol[i][j] = 0;
        }
        pertenece[i] = &verAux->dato;
        datos[i] = &verAux->dato;
        verAux = verAux->sig;
    }
    init_matriz_distancia(matAdyacencia, pertenece);
    
    for (long aristas = 1; aristas < size(); ++aristas){
        min = INF;
        for (long i = 0; i < size(); ++i){
            for (long j = 0; j < size(); ++j){
                if (min > matAdyacencia[i][j] && matAdyacencia[i][j] != 0 &&
                    *pertenece[i] != *pertenece[j]){
                        min = matAdyacencia[i][j];
                        nodoA.second = i;
                        nodoA.first = pertenece[i];
                        nodoB.second = j;
                        nodoB.first = pertenece[j];
                }
            }
        }
        if (*pertenece[nodoA.second] != *pertenece[nodoB.second]){
            matArbol[nodoA.second][nodoB.second] = min;
            matArbol[nodoB.second][nodoA.second] = min;

            pair<T*, long> tmp = nodoB;
            pertenece[nodoB.second] = pertenece[nodoA.second];
            for (long k = 0; k < size(); ++k){
                if (*pertenece[k] == *tmp.first)
                    pertenece[k] = pertenece[nodoA.second];
            }
        }
    }
    long pesoTotal = 0;
    long j;
    for (long i = 0; i < size(); ++i){
        for (j = i + 1; j < size(); ++j){
            if (matArbol[i][j] != 0)
                cout << *datos[i] << "->" << *datos[j]
                     << " | peso arista: " << matArbol[i][j] << endl;
            pesoTotal += matArbol[i][j];
        }
        if (j > i + 2)
            cout << endl;
    }
    cout << "peso total del árbol resultante: " << pesoTotal << endl;
}

template <typename T>
void Grafo<T>::init_matriz_distancia(vector<vector<long>>&matriz, T* arrDatos[])
{
    Vertice<T> *verAux;
    Arista<T> *ariAux;
    long columna = 0;

    // Inicializa la matriz de distancias menos la
    // diagonal
    verAux = raiz;
    for (long fila = 0; verAux != nullptr; ++fila){
        ariAux = verAux->aristaAdy;
        while (ariAux != nullptr){
            columna = buscar_pos_vertice(arrDatos, ariAux->verticeAdy);
            if (columna != -1)
                matriz[fila][columna] = ariAux->peso;
            ariAux = ariAux->sig;
        }
        verAux = verAux->sig;
    }
}

template<typename T>
void Grafo<T>::prim()
{
    vector<vector<long>> matAdyacencia(size());
    long matArbol[size()][size()];
    T* datos[size()];
    vector<long> lineas;
    long padre;
    long hijo;
    long min;
    Vertice<T>* verAux = raiz;

    if (dirigido == DIRIGIDO){
        cout << "No es posible realizar este algoritmo" << endl
             << "ya que se requiere de un grafo NO DIRIGIDO" << endl;
        return;
    }

    // Se inicializan ambas matrices
    for (long i = 0; i < size(); ++i){
        for (long j = 0; j < size(); ++j){
            matAdyacencia[i].push_back(INF);
            if (i == j)
                matAdyacencia[i][i] = 0;
            matArbol[i][j] = 0;
        }
        datos[i] = &verAux->dato;
        verAux = verAux->sig;
    }
    init_matriz_distancia(matAdyacencia, datos);

    padre = 0;
    hijo = 0;
    while (lineas.size() + 1 < size()){
        padre = hijo;
        lineas.push_back(padre);
        for (long i = 0; i < size(); ++i)
            matAdyacencia[i][padre] = INF;
        min = INF;
        for (long i = 0; i < lineas.size(); ++i)
            for(long j = 0; j < size(); ++j){
                if (min > matAdyacencia[i][j] && matAdyacencia[i][j]){
                    min = matAdyacencia[i][j];
                    padre = i;
                    hijo = j;
                }
            }
        matArbol[padre][hijo] = min;
        matArbol[hijo][padre] = min;
    }

    long pesoTotal = 0;
    long j;
    for (long i = 0; i < size(); ++i){
        for (j = i + 1; j < size(); ++j){
            if (matArbol[i][j] != 0)
                cout << *datos[i] << "->" << *datos[j]
                     << " | peso arista: " << matArbol[i][j] << endl;
            pesoTotal += matArbol[i][j];
        }
        if (j > i + 2)
            cout << endl;
    }
    cout << "peso total del árbol resultante: " << pesoTotal << endl;
}

#endif // GRAPH_H