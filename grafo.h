#ifndef GRAFO_H
#define GRAFO_H

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

//// VERTICE ////
template<typename T>
class Vertice
{
    Vertice<T> *siguiente;
    Arista<T> *aristaAdy;
    T dato;
    friend class Grafo<T>;
public:
    Vertice(){}
    ~Vertice(){}
};

//// ARISTA ////
template<typename T>
class Arista
{
    double peso;
    Arista<T>* siguiente;
    Vertice<T> *verticeAdy;

    friend class Grafo<T>;
public:
    Arista(){}
    ~Arista(){}
};

//// GRAFO ////
template<typename T>
class Grafo
{
    Vertice<T>* m_raiz;
    Arista<T>* getAristaFinal(Vertice<T>& vert);
    bool m_dirigido;

    void insertar_arista_p(Vertice<T> *origen, Vertice<T> *destino, double peso);
    void eliminar_arista_p(Vertice<T>* origen, Vertice<T>* destino);
    long buscar_nodo(Nodo<T> arrNodo[], Vertice<T> *&vertice);
    long buscar_pos_vertice(T* arrVer[], Vertice<T>*& dato);
    void imprimir_mejor_camino(Nodo<T>arrNodo[]);
    void init_matriz_distancia(vector<vector<long>> &matriz, T *arrDatos[]);
public:
    Grafo(bool dirigido = DIRIGIDO) : m_raiz(nullptr), m_dirigido(dirigido) {}
    ~Grafo(){ limpiar(); }

    Vertice<T>* getVertice(T dato);
    void insertar_arista(T origen, T destino, double peso);
    bool vacio();
    int tamanio();
    void insertar_vertice(T dato);
    void lista_adyacencia();
    void limpiar();
    void eliminar_arista(T origen, T destino);
    void eliminar_vertice(T vertice);
    void dijkstra(T origen);
    void floyd();
    void kruskal ();
    void prim();
};


/// DEFINICIONES DE MÉTODOS ///

// Retorna "true" si el grafo aún no tiene vértices introducidos
// y retorna false en caso contrario
template<typename T>
bool Grafo<T>::vacio()
{ return m_raiz == nullptr; }

template<typename T>
int Grafo<T>::tamanio()
{
    Vertice<T> *aux;
    int cont = 0;
    
    aux = m_raiz;
    while (aux != nullptr){
        aux = aux->siguiente;
        ++cont;
    }
    return cont;
}

// Retorna el vértice existente en el grafo en el que
// se encuentra un dato dado
template<typename T>
Vertice<T>* Grafo<T>::getVertice(T dato)
{
    Vertice<T> *aux;
    aux = m_raiz;
    while (aux != nullptr){
        if (aux->dato == dato)
            return aux;
        
        aux = aux->siguiente;
    }
    return nullptr;
}

// Se inserta in vértice nuevo, siendo el siguiente 
// y último en la lista de vértices
template<typename T>
void Grafo<T>::insertar_vertice(T data)
{
    Vertice<T> *nuevo = new Vertice<T>;
    nuevo->dato = data;
    nuevo->aristaAdy = nullptr;
    nuevo->siguiente = nullptr;
    if (vacio())
        m_raiz = nuevo;
    else if(getVertice(data) != nullptr){
        cout << "El elemento ya ha sido insertado" << endl; 
        delete nuevo;
    }
    else{
        Vertice<T>* aux;
        aux = m_raiz;
        while(aux->siguiente != nullptr)
            aux = aux->siguiente;

        aux->siguiente = nuevo;
    }
}

// Inserta una nueva arista en la lista de aristas de un vértice existente
// conectándolo además con otro vértice
template <typename T>
void Grafo<T>::insertar_arista_p(Vertice<T> *origen, Vertice<T> *destino, double _peso)
{
    bool repetido = false;
    Arista<T> *nueva = new Arista<T>;
    Arista<T> *aux;
    Vertice<T>* vertAux;

    nueva->peso = _peso;
    nueva->siguiente = nullptr;
    nueva->verticeAdy = nullptr;
    aux = origen->aristaAdy;
    
    if(aux == nullptr){
        origen->aristaAdy = nueva;
        nueva->verticeAdy = destino;
    }
    else{
        while(aux->siguiente != nullptr){
            if (aux->verticeAdy == destino){
                cout << "La arista ya existe" << endl;
                repetido = true;
                delete nueva;
                break;
            }
            aux = aux->siguiente;
        }
        
        if (!repetido && aux->verticeAdy != destino){
            aux->siguiente = nueva;
            nueva->verticeAdy = destino;
        }
    }
}

// Se utiliza para insertar una nueva arista en el grafo conectándo así
// a dos vértices existentes
template <typename T>
void Grafo<T>::insertar_arista(T origen, T destino, double peso)
{
    Vertice<T>* origenTmp = getVertice(origen);
    Vertice<T> *destinoTmp = getVertice(destino);
    insertar_arista_p(origenTmp, destinoTmp, peso);

    if (m_dirigido == NO_DIRIGIDO)
        insertar_arista_p(destinoTmp, origenTmp, peso);
}

// Imprime todas las conexiones de cada vértice con
// el resto de vértices
template<typename T>
void Grafo<T>::lista_adyacencia()
{
    cout << "Conexiones (vértice -> aristas):" << endl;
    Vertice<T> *vertAux;
    Arista<T> *arisAux;
    vertAux = m_raiz;
    while(vertAux != nullptr){
        arisAux = vertAux->aristaAdy;
        cout << vertAux->dato << "->";
        while (arisAux != nullptr){
            cout << arisAux->verticeAdy->dato
                 << "->";
            arisAux = arisAux->siguiente;
        }
        cout << endl;
        vertAux = vertAux->siguiente;
    }
}

// Obtiene la arista final de la lista de aristas
// de un vértice
template <typename T>
Arista<T> *Grafo<T>::getAristaFinal(Vertice<T> &vert)
{
    Arista<T>* aux;
    aux = vert->aristaAdy;
    if (aux == nullptr)
        return nullptr;
    while (aux->siguiente != nullptr)
        aux = aux->siguiente;    
    return aux;
}

// Borra todos los vértices del grafo
template<typename T>
void Grafo<T>::limpiar()
{
    Vertice<T> *vertAux;
    Arista<T> *arisAux;
    while (m_raiz != nullptr){
        vertAux = m_raiz;
        while (vertAux->aristaAdy != nullptr){
            arisAux = vertAux->aristaAdy;
            vertAux->aristaAdy = vertAux->aristaAdy->siguiente;
            delete arisAux;
        }
        m_raiz = m_raiz->siguiente;
        delete vertAux;
    }
}

// Elimina una de las aristas de la lista de aristas
// de los dos vértices que conecta
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
        origen->aristaAdy = aristaActual->siguiente;
        delete aristaActual;
    }
    else{
        while(aristaActual != nullptr){
            if (aristaActual->verticeAdy == destino){
                encontrado = true;
                aristaAnterior->siguiente = aristaActual->siguiente;
                delete aristaActual;
                break;
            }
            aristaAnterior = aristaActual;
            aristaActual = aristaActual->siguiente;
        }
        if(!encontrado)
            cout << "Los vértices no están conectados" << endl;
    }
}

// Elimina una de las aristas de la lista de aristas
// de los dos vértices que conecta
template<typename T>
void Grafo<T>::eliminar_arista(T origen, T destino)
{
    Vertice<T>* origenTmp = getVertice(origen);
    Vertice<T> *destinoTmp = getVertice(destino);
    eliminar_arista_p(origenTmp, destinoTmp);
    if (m_dirigido == NO_DIRIGIDO)
        eliminar_arista_p(destinoTmp, origenTmp);
}

// Elimina un vértice de la lista de vértices y además
// elimina todas sus aristas de su lista de aristas
// y elimina todas sus aristas adyacentes de la lista
// de aristas de otros vértices
template<typename T>
void Grafo<T>::eliminar_vertice(T datoOrigen)
{
    Vertice<T> *vertice = getVertice(datoOrigen);
    Vertice<T> *vertActual;
    Vertice<T> *vertAnterior;
    Arista<T> *aristaAux;
    vertActual = m_raiz;
    // Elimina las conexiones entre el vértice a 
    // borrar y el resto de vértices
    while(vertActual != nullptr){
        aristaAux = vertActual->aristaAdy;
        while(aristaAux != nullptr){
            if (aristaAux->verticeAdy == vertice){
                eliminar_arista(vertActual->dato, aristaAux->verticeAdy->dato);
                break;
            }
            aristaAux = aristaAux->siguiente;
        }
        vertActual = vertActual->siguiente;
    }
    
    //El vertice a borrar es la raíz
    vertActual = m_raiz;
    if (m_raiz == vertice)
        m_raiz = m_raiz->siguiente;
    
    //Caso contrario
    else{
        while(vertActual != vertice){
            vertAnterior = vertActual;
            vertActual = vertActual->siguiente;
        }
        vertAnterior->siguiente = vertActual->siguiente;
    }

    //Se borran las aristas del vértice
    while (vertActual->aristaAdy != nullptr){
        aristaAux = vertActual->aristaAdy;
        vertActual->aristaAdy = vertActual->aristaAdy->siguiente;
        delete aristaAux;
    }
    //Se borra el vértice
    delete vertActual;
}

// Se utiliza para la comparación en la cola de prioridad
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
    for (long i = 0; i < tamanio(); ++i){
        if (arrNodo[i].actual->dato == vertice->dato)
            return i;
    }
    return -1;
}

template <typename T>
long Grafo<T>::buscar_pos_vertice(T* arrVer[], Vertice<T>*& dato)
{
    for (long i = 0; i < tamanio(); ++i){
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

    for (size_t i = 0; i < tamanio(); ++i){
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

//// ALGORITMOS ////

// Realiza el algoritmo de dijkstra en un vértice dado
// e imprime los recorridos
template <typename T>
void Grafo<T>::dijkstra(T datoOrigen)
{
    Vertice<T>* origen = getVertice(datoOrigen);
    Nodo<T> arrNodo[tamanio()];
    priority_queue<Nodo<T>*, vector<Nodo<T>*>, cmp<T>> cola;
    // Representa "u"
    Nodo<T> *nodoActual;
    // Representa "v"
    Nodo<T> *nodoAdyacente;
    Vertice<T>* vertAux;
    Arista<T>* arisAux;
    long pos;

    // Se inicializan todos los datos del array de nodos
    vertAux = m_raiz;
    for (size_t i = 0; i < tamanio(); ++i){
        if (vertAux->dato == origen->dato){
            pos = long(i);
            arrNodo[i].acumulado = 0;
        }
        arrNodo[i].actual = vertAux;
        vertAux = vertAux->siguiente;
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
            arisAux = arisAux->siguiente;
        }
    }
    imprimir_mejor_camino(arrNodo);
}

// Realiza el algoritmo de floyd en todos los vértices
// e imprime los recorridos
template <typename T>
void Grafo<T>::floyd()
{
    vector<vector<long>> matDistancias(tamanio());
    Vertice<T>* matRecorrido[tamanio()][tamanio()];
    Vertice<T>* verAux = m_raiz;
    T* arrDatos[tamanio()]; 
    long posAux;

    // Inicializa la matriz de recorrido
    for(long i = 0; i < tamanio(); i++){
        for (long j = 0; j < tamanio(); j++){
            matDistancias[i].push_back(INF);
            if (i != j)
                matRecorrido[j][i] = verAux;
            else{
                matDistancias[i][i] = 0;
                matRecorrido[j][i] = nullptr;
            }
        }
        arrDatos[i] = &verAux->dato;
        verAux = verAux->siguiente;
    }
    init_matriz_distancia(matDistancias, arrDatos);

    // Se ejecuta el algoritmo Floyd
    for(int k = 0; k < tamanio(); k++){
        for(int i = 0; i < tamanio(); i++)
            for(int j = 0; j < tamanio(); j++){
                unsigned long dt = matDistancias[i][k] + matDistancias[k][j];
                if (matDistancias[i][j] > dt && j != k && i != k){
                    matDistancias[i][j] = dt;
                    matRecorrido[i][j] = getVertice(*arrDatos[k]);
                }
            }
    }
    // Se imprimen los recorridos de cada vértice
    for (int i = 0; i < tamanio(); i++){
        for (int j = 0; j < tamanio(); ++j){
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

// Realiza el algoritmo de kruskal en todos los vértices
// e imprime los recorridos
template <typename T>
void Grafo<T>::kruskal()
{
    vector<vector<long>> matAdyacencia(tamanio());
    long matArbol[tamanio()][tamanio()];
    T* pertenece[tamanio()];
    T *datos[tamanio()];
    pair<T*, long> nodoA;
    pair<T*, long> nodoB;
    long min;
    Vertice<T>* verAux = m_raiz;

    if (m_dirigido == DIRIGIDO){
        cout << "No es posible realizar este algoritmo" << endl
             << "ya que se requiere de un grafo NO DIRIGIDO" << endl;
        return;
    }

    // Se inicializan ambas matrices
    for (long i = 0; i < tamanio(); ++i){
        for (long j = 0; j < tamanio(); ++j){
            matAdyacencia[i].push_back(INF);
            if (i == j)
                matAdyacencia[i][i] = 0;
            matArbol[i][j] = 0;
        }
        pertenece[i] = &verAux->dato;
        datos[i] = &verAux->dato;
        verAux = verAux->siguiente;
    }
    init_matriz_distancia(matAdyacencia, pertenece);
    
    for (long aristas = 1; aristas < tamanio(); ++aristas){
        min = INF;
        for (long i = 0; i < tamanio(); ++i){
            for (long j = 0; j < tamanio(); ++j){
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
            for (long k = 0; k < tamanio(); ++k){
                if (*pertenece[k] == *tmp.first)
                    pertenece[k] = pertenece[nodoA.second];
            }
        }
    }
    long pesoTotal = 0;
    long j;
    for (long i = 0; i < tamanio(); ++i){
        for (j = i + 1; j < tamanio(); ++j){
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
    verAux = m_raiz;
    for (long fila = 0; verAux != nullptr; ++fila){
        ariAux = verAux->aristaAdy;
        while (ariAux != nullptr){
            columna = buscar_pos_vertice(arrDatos, ariAux->verticeAdy);
            if (columna != -1)
                matriz[fila][columna] = ariAux->peso;
            ariAux = ariAux->siguiente;
        }
        verAux = verAux->siguiente;
    }
}

// Realiza el algoritmo de prim en todos los vértices
// e imprime los recorridos
template<typename T>
void Grafo<T>::prim()
{
    vector<vector<long>> matAdyacencia(tamanio());
    long matArbol[tamanio()][tamanio()];
    T* datos[tamanio()];
    vector<long> lineas;
    long padre;
    long hijo;
    long min;
    Vertice<T>* verAux = m_raiz;

    if (m_dirigido == DIRIGIDO){
        cout << "No es posible realizar este algoritmo" << endl
             << "ya que se requiere de un grafo NO DIRIGIDO" << endl;
        return;
    }

    // Se inicializan ambas matrices
    for (long i = 0; i < tamanio(); ++i){
        for (long j = 0; j < tamanio(); ++j){
            matAdyacencia[i].push_back(INF);
            if (i == j)
                matAdyacencia[i][i] = 0;
            matArbol[i][j] = 0;
        }
        datos[i] = &verAux->dato;
        verAux = verAux->siguiente;
    }
    init_matriz_distancia(matAdyacencia, datos);

    padre = 0;
    hijo = 0;
    while (lineas.size() + 1 < tamanio()){
        padre = hijo;
        lineas.push_back(padre);
        for (long i = 0; i < tamanio(); ++i)
            matAdyacencia[i][padre] = INF;
        min = INF;
        for (long i = 0; i < lineas.size(); ++i)
            for(long j = 0; j < tamanio(); ++j){
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
    for (long i = 0; i < tamanio(); ++i){
        for (j = i + 1; j < tamanio(); ++j){
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

#endif // GRAFO_H