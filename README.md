# Visual_Odometry
Progetto SGN: Visual Odometry

# Abbozzo di Relazione

## Contenuti:
* [1. Introduzione](#1-introduzione)
* [2. Preprocessing dell'Immagine](#2-preprocessing-dell'Immagine)
* [3. Feature Detecting](#3-feature-detecting)
* [4. Feature Matching](#4-feature-matching)
* [5. Motion Estimation](#5-motion-estimation)
* [6. Risultati](#6-risultati)
* [7. visual_odometry pkg](#7-visual_odometry-pkg)
* [8. Conclusioni](#8-conclusioni)

## 1) Introduzione

![alt text](/docs/img_relazione/VO_steps.png)

## 2) Preprocessing dell'Immagine

## 3) Feature Detecting

## 4) Feature Matching

## 5) Motion Estimation 

## 6) Risultati 

## 7) visual_odometry pkg

## 8) Conclusioni








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
