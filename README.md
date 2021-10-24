# Visual_Odometry (Bozza)
Progetto SGN: Visual Odometry

## Contenuti:
* [1. Introduzione](#1-introduzione)
* [2. Visual Odometry](#2-visual-odometry)
* [3. Preprocessing dell'Immagine](#3-preprocessing-dellimmagine)
* [4. Feature Detection and Matching](#4-feature-detection-and-matching)
* [5. Motion Estimation](#5-motion-estimation)
* [6. Risultati](#6-risultati)
* [7. visual_odometry pkg](#7-visual_odometry-pkg)
* [8. Conclusioni](#8-conclusioni)
* [9. Bibliografia](#9-bibliografia)

## 1) Introduzione
L'obiettivo del progetto è quello di implementare uno script MATLAB di Visual Odometry su ROS, con nodi implementati in C++. L'obiettivo è dunque quello di verificare non solo la correttezza dello script originale, ma di valutare se OpenCV raggiunge risultati migliori del toolbox di MATLAB.
### 1.1) Camera Modeling: Perspective Camera Model
Pinhole camera projection system:

![alt text](/docs/img_relazione/camera_model.png)

## 2) Visual Odometry
La Visual Odometry consiste nel determinare il movimento della camera da una sequenza di immagini. La camera è rigidamente attaccata all'agente che ne permette il movimento.

![alt text](/docs/img_relazione/vo_example.png)

Impostiamo il problema in maniera formale. Nel caso della **Monocular VO**, ad ogni istante discreto di tempo k, avremo un set di immagini I<sub>0:k</sub> = {I<sub>0</sub>, ... , I<sub>k-1</sub>, I<sub>k</sub>}. Si definisce inoltre un set di pose della camera C<sub>0:k</sub> = {C<sub>0</sub>, ...,C<sub>k-1</sub>, C<sub>k</sub>}. 
La trasformazione di coordinate tra due pose della camera ad istanti adiacenti k-1 e k, è definita come:

![alt text](/docs/img_relazione/t_k.png)

Dove R<sub>k,k-1</sub> e t<sub>k,k-1</sub> sono rispettivamente la matrice di rotazione e il vettore di traslazione tra le due pose C<sub>k-1</sub> e C<sub>k</sub>. E' possibile dunque ottenere le pose della camera concatenando le trasformazioni ricavate ad ogni passo, attraverso la relazione 
C<sub>k</sub> = C<sub>k-1</sub> T<sub>k,k-1</sub>.

L'obiettivo principale dunque della Visual Odometry è ricavare le trasformazioni T<sub>k,k-1</sub> a partire dalle immagini I<sub>k</sub> e I<sub>k-1</sub>, per poi ottenere C<sub>k</sub>.

![alt text](/docs/img_relazione/vo_problem.png)

Per fare ciò, l'algoritmo si articola in diversi step:

![alt text](/docs/img_relazione/VO_steps.png).

## 3) Preprocessing dell'Immagine

### 3.1) Undistort Image
### 3.2) Convert in Grey Scale

## 4) Feature Detection and Matching

### 4.1) Feature Dection

### 4.2) Feature Matching
TO DO: 
- Parlare del knnMatch() e della sua differenza con match().
- Lowe Ratio.

### 4.3) Implementazione in OpenCV
## 5) Motion Estimation 
Lo step piu' importante della Visual Odometry consiste nella Motion Estimation, basata sull'estrazione delle matrici di Rotazione R<sub>k,k-1</sub> e dei vettori di traslazione t<sub>k,k-1</sub>. Gli approcci per estrarre la trasformazione di coordinate sono:

1. 2D to 2D: Le feature estratte f<sub>k-1</sub> e f<sub>k</sub> sono espresse nel piano immagine (2D).

2. 3D to 3D: Le feature estratte f<sub>k-1</sub> e f<sub>k</sub> sono espresse in coordinate 3D. Occorre dunque triangolare i punti ad ogni istante. Utili nel caso si stia implementando una Stereo VO.

3. 3D to 2D: Le feature estratte f<sub>k-1</sub> e f<sub>k</sub> sono espresse rispettivamente in coordinate 3D e nel piano immagine 2D.

Dato che la Visual Odometry è di tipo Monocular, l'approccio 2D to 2D è consigliabile, in quanto evita un'ulteriore triangolazione.

### 5.1) Essential Matrix
L'Essential Matrix racchiude i parametri del moto della camera, a meno di un fattore di scala relativo al vettore di translazione. In particolare, e' possibile scrivere:

![alt text](/docs/img_relazione/essential.png)

Dove l'uguaglianza evidenza la non conoscenza del fattore di scala. Il vettore t<sub>k</sub> e' in forma di matrice antisimmetrica.

L'Essential Matrix verrà calcolata dunque dalle feature 2D individuate dallo step precedente, aggiungendo un ulteriore vincolo, ovvero **epipolar constraint**. Questo vincolo puo' essere espresso matematicamente come:

p<sup>'T</sup> E p = 0.

Dove p<sup>'</sup> e p sono rispettivamente le feature individuate nell'immagine I<sub>k</sub> e I<sub>k-1</sub>.

![alt text](/docs/img_relazione/epipolar.png)

Per calcolare L'Essential Matrix esistono diversi algoritmi. Quello usato consiste in un algoritmo ad 8 punti, usando **RANSAC**. Quest'ultimo algoritmo è fondamentale per la Visual Odometry. Esso infatti reietta gli outliers, rendendo la stima della posa efficace.

In OpenCV, cio' viene implementato dal comando:

```
Mat E = findEssentialMat(kP_converted.Kpoints1, kP_converted.Kpoints2, cameraMatrix, RANSAC, 0.999, 1.0, RANSAC_mask);
```

Il comando prende in input i KeyPoint individuato dallo step di Feature Matching, i parametri intrinsechi della camera, il metodo da usare per il calcolo della matrice (con relativi parametri) ed infine una `RANSAC_MASK`. Questa di fatto è un vettore di lunghezza pari ai vettori di KeyPoint e contiene 1 o 0, individuando quali di questi KeyPoint siano outlier o inlier.

![alt text](/docs/img_relazione/ransac.png)

### 5.2) Estrazione di R e t
Trovata l'Essential Matrix, l'obiettivo adesso e' quello di estrarre la matrice di rotazione R e il vettore di traslazione t. 
In generale esistono 4 soluzioni. Tuttavia, esse vengono scartate attraverso la triangulazione di un singolo punto. 

Le 4 soluzioni sono:

![alt text](/docs/img_relazione/4soluz.png)

Dove la matrice W e':

![alt text](/docs/img_relazione/W.png)

In OpenCV, l'estrazione di R e t è implementata dal comando:

```
recoverPose(E, kP_converted.Kpoints1, kP_converted.Kpoints2, cameraMatrix, R, t, RANSAC_mask);
```

Il comando `recoverPose()` prende in input la matrice essentiale, i KeyPoints, la Mask fornita dall'algoritmo RANSAC e restituisce `R` e `t`.
### 5.3) Scale Factor

### 5.4) Riassunto dell'algoritmo proposto

![alt text](/docs/img_relazione/vo_2d.png)
## 6) Risultati 

## 7) visual_odometry pkg

### 7.1) Istruzioni per la compilazione e l'esecuzione del pkg

Per la compilazione e l'esecuzione del pkg, sono necessari i seguenti prerequisiti:
- ROS Melodic (Ubuntu 18.04).
- `vision_opencv`: Permette l'interfaccia tra OpenCV e ROS.
- OpenCV.
- `opencv_contrib` (Necessario per usare l'algoritmo SURF).

#### Guida per l'installazione di OpenCV e opencv_contrib:
**SOLUZIONE NON DEFINITVA**: Alla fine dell'esecuzione del nodo, va in errore, essendoci un conflitto tra OpenCV usato da `cv_bridge` (3.2.0, default in ROS Melodic) e OpenCV 4 usato per `xfeatures2d`.
Bisogna trovare un'altra soluzione, ma al momento va bene per lavorare sul codice.
La soluzione è presa da questo [link](https://answers.ros.org/question/312669/ros-melodic-opencv-xfeatures2d/).

Per l'installazione di OpenCV 4 con opencv_contrib, digitare nel terminale:

```
mkdir ~/opencv_build && cd ~/opencv_build
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```
Queste 3 istruzioni creano una cartella `opencv_build` dentro cui vengono scaricati `opencv` e `opencv_contrib`.

Dopo di ciò, dal terminale:

```
cd ~/opencv_build/opencv
mkdir build && cd build
```
Setup OpenCV build con CMake:

```
cmake -D CMAKE_BUILD_TYPE=RELEASE \  
        -D CMAKE_INSTALL_PREFIX=/usr/local \  
        -D INSTALL_C_EXAMPLES=ON \  
        -D INSTALL_PYTHON_EXAMPLES=ON \  
        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \  
        -D BUILD_EXAMPLES=ON .. \  
        -D OPENCV_ENABLE_NONFREE=ON ..
```
Compiliamo:

```
make -j8
```
Installiamo:

```
sudo make install
```
L'installazione è completa.


Disinstallare OpenCV:

```
sudo rm /usr/local/{bin,lib}/*opencv* 
```

### 7.2) Struttura del pkg
### 7.3) Versioni diverse
E' possibile strutturare un nodo di ROS in vari modi. Il primo, chiamato **Procedural ROS Node**, che consiste in un loop `while(ros::ok())` ed in cui il flusso dei dati e delle istruzioni è evidente e leggibile. Il secondo invece, chiamato **Object-Oriented ROS Node**, consiste nel dichiarare una classe i cui attributi privati sono i componenti principali di un nodo (`ros::NodeHandle`, `ros::Subscriber`, etc.) e le callback sono i metodi della classe. 

1. **Procedural ROS Node**: Consiste in un loop `while(ros::ok())` la cui frequenza viene decisa dall'oggetto `ros::Rate`. Le callBack sono dichiarate fuori dal main. 
Il codice risulta leggibile e si segue bene il flusso delle istruzioni. Il codice tuttavia richiede variabili globali ed inoltre risulta poco riutilizzabile in altri nodi.

2. **Object-Oriented ROS Node**: L'intero nodo viene implementato in una classe, in cui i componenti principali di un nodo (`ros::NodeHandle`, `ros::Subscriber`, etc.) sono i componenti `private` della classe, mentre le callBack consistono nei metodi. 
Per riusare il codice in altri nodi, basta dichiarare l'oggetto della classe progettata. 
Se si è interessati implementare il loop del **Procedural ROS Node**, è possibile dichiarare nei componenti `private` l'oggetto `ros::Timer`, che di fatto sostituisce il `ros::Rate`. 

Saranno disponibili dunque entrambe le versioni per la Visual Odometry. L'idea e' quella di poter facilmente usare il codice ad esempio dentro un Filtro di Navigazione (ex. EKF) e dunque basterebbe dichiarare l'oggetto `Visual_Odometry` e riusare tutto il codice agilmente.

Una struttura generale può essere del tipo:

```
class MyNode
{
    public:
        MyNode():
            nh{},
            pub(nh.advertise<sensor_msgs::JointState>("js", 5)),
            sub(nh.subscribe("topic", 1000, &MyNode::callback, this)),
            timer(nh.createTimer(ros::Duration(0.1), &MyNode::main_loop, this))
         {
         }

         void callback(const sensor_msgs::JointState & js) const
         {
         }

         void main_loop(const ros::TimerEvent &) const
         {
             pub.publish(sensor_msgs::JointState{});
         }

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::Timer timer;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nodename");
    MyNode node;
    ros::spin();
    return 0;
}
```

## 8) Conclusioni

## 9) Bibliografia
[1] Visual Odometry Tutorial, [parte 1](http://rpg.ifi.uzh.ch/docs/VO_Part_I_Scaramuzza.pdf), [parte 2](http://rpg.ifi.uzh.ch/docs/VO_Part_II_Scaramuzza.pdf).
-------------------------------------------------------------------------------------
# Appunti per sviluppo del codice

#### Flow Chart:
![alt text](/docs/img_relazione/vo_flowchart_ns.png)

#### Task List:
- [ ] image_transport compressed.
- [ ] Provare cv_bridge con opencv_contrib 3.2.0.
- [ ] Scrivere la relazione.
- [ ] World Points su Rviz con riferimenti corretti. 


https://forum.opencv.org/t/easy-way-to-compare-quality-of-keypoints/4939/3