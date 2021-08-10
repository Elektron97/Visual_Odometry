# Visual_Odometry (Bozza)
Progetto SGN: Visual Odometry

## Contenuti:
* [1. Introduzione](#1-introduzione)
* [2. Visual Odometry](#2-visual-odometry)
* [3. Preprocessing dell'Immagine](#3-preprocessing-dellimmagine)
* [4. Feature Detecting](#4-feature-detecting)
* [5. Feature Matching](#5-feature-matching)
* [6. Motion Estimation](#6-motion-estimation)
* [7. Risultati](#7-risultati)
* [8. visual_odometry pkg](#8-visual_odometry-pkg)
* [9. Conclusioni](#9-conclusioni)
* [10. Bibliografia](#9-bibliografia)

## 1) Introduzione
## 2) Visual Odometry
La Visual Odometry consiste nel determinare il movimento della camera da una sequenza di immagini. La camera è rigidamente attaccata all'agente che ne permette il movimento.

![alt text](/docs/img_relazione/vo_example.png)

Impostiamo il problema in maniera formale. Nel caso della **Monocular VO**, ad ogni istante discreto di tempo k, avremo un set di immagini I<sub>0:k</sub> = {I<sub>0</sub>, ... , I<sub>k-1</sub>, I<sub>k</sub>}. Si definisce inoltre un set di pose della camera C<sub>0:k</sub> = {C<sub>0</sub>, ...,C<sub>k-1</sub>, C<sub>k</sub>}. 
La trasformazione di coordinate tra due pose della camera ad istanti adiacenti k-1 e k, è definita come:
![alt text](/docs/img_relazione/t_k.png)

Dove R<sub>k,k-1</sub> e t<sub>k,k-1</sub> sono rispettivamente la matrice di rotazione e il vettore di traslazione tra le due pose C<sub>k-1</sub> e C<sub>k</sub>. E' possibile dunque ottenere le pose della camera concatenando le trasformazioni ricavate ad ogni passo, attraverso la relazione 
C<sub>k</sub> = C<sub>k-1</sub> T<sub>k,k-1</sub>.

L'obiettivo principale dunque della Visual Odometry è ricavare le trasformazioni T<sub>k,k-1</sub> a partire dalle immagini I<sub>k</sub> e I<sub>k-1</sub>, per poi ottenere C<sub>k</sub>.

Per fare ciò, l'algoritmo si articola in diversi step, ognuno di questi spiegato nelle sezioni apposite:
![alt text](/docs/img_relazione/VO_steps.png).
## 3) Preprocessing dell'Immagine

## 4) Feature Detecting

## 5) Feature Matching

## 6) Motion Estimation 

## 7) Risultati 

## 8) visual_odometry pkg

## 9) Conclusioni

## 10) Bibliografia
[1] Visual Odometry Tutorial, [parte 1](http://rpg.ifi.uzh.ch/docs/VO_Part_I_Scaramuzza.pdf), [parte 2](http://rpg.ifi.uzh.ch/docs/VO_Part_II_Scaramuzza.pdf).






-------------------------------------------------------------------------------------
### Steps per la Visual Odometry:
![alt text](/docs/img_relazione/VO_steps.png)

- Undistort e conversione in scala di grigi.
- Detect Features.
- Match Features.
- Essential Matrix.
- Reject Outlier.
- Decompose Essential Matrix -> R<sub>k</sub> e t<sub>k</sub>.
- Triangulation -> World Points

## Appunti sulle Trasformazioni di Coordinate

![alt text](/docs/appunti/schema1.jpg)
![alt text](/docs/appunti/schema2.jpg)

### Tabella di conversione tra teoria, MATLAB e codice:

Legend: 
- R_{ab} means Rotation from a to b. k_1 prev frame, k curr frame.
- t_{a, b}^a -> da a a b in coordinate {a}.

| Theory           | MATLAB        | ROS               |  Function             |
| -----------------|:-------------:| -----------------:| ---------------------:|
| R_{w, k_1}       | Rotm          |   orientation     | absPose               |
| t_{w, k_1}^w     | tran          |   location        | absPose               |
| R_{k, k_1}       | orient        |     R             | recoverPose           |
| t_{k_1, k}^k_1   | loc           |     t             | recoverPose           |
| world_points{k}  | world_points  | world_points      | triangPoints          |
| world_pointsW{w} | world_pointsW | world_pointsW     | absPose               |

## Prime Performance

![alt text](/docs/risultati/err_pos.png)

- 2-3 metri di errore: troppo alto!

#### Task List: 
- [ ] image_transport compressed.
- [ ] provare cv_bridge con opencv_contrib 3.2.0.
- [x] World Position.
- [x] Inserire la Rbc nell'upload data.
- [x] Fare il Ground Truth.
- [x] Finire e commentare il read me sulle matrici di rotazione.
- [x] Show Result.
- [ ] Inserire booleani di sicurezza (success, fail_detection).
- [x] Stima velocita' lineare ed angolare per derivazione.
- [x] Reprojection Error.
- [ ] Reprojection in Curr frame troppo alto, rivedere.
- [ ] Plottare PointCloud2 i worldPoints.
