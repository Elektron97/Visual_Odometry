# Visual_Odometry
Progetto SGN: Visual Odometry

## Appunti sulle Trasformazioni di Coordinate

![alt text](/docs/schema1.jpg)
![alt text](/docs/schema2.jpg)

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

#### Task List: 
- [ ] image_transport compressed.
- [ ] provare cv_bridge con opencv_contrib 3.2.0.
- [x] World Position.
- [x] Inserire la Rbc nell'upload data.
- [x] Fare il Ground Truth.
- [x] Finire e commentare il read me sulle matrici di rotazione.
- [x] Show Result.
- [ ] Inserire booleani di sicurezza (success, fail_detection).
- [ ] Stima velocita' lineare ed angolare per derivazione.
