# ns3Ros2Bridge



## NS-3.38

- [ ] Contient les scripts et sources nécessaires pour la partie ns-3 du projet 
- [ ] Configurer et vérifier le fonctionnement de ns-3 

```
cd ns-3/ns-3.38
sudo ./dependencies.bsh
./install.bsh
./test.bsh 
```

- [ ] Compiler de nouveaux changements

```
cd ns-3/ns-3.38
./compile.bsh
```

- [ ] Lancer le projet

```
cd ns-3/ns-3.38
./run.bsh
```

## ROS2 Humble

- [ ] Contient les scripts et sources nécessaires pour la partie ros2 du projet 
- [ ] Configurer et vérifier le fonctionnement de ros2 

```
cd ros2-humble
sudo ./dependencies.bsh
./install.bsh
```

- [ ] Compiler de nouveaux changements

```
cd ros2-humble
./compile.bsh
```

- [ ] Lancer le projet

```
cd ros2-humble
sudo ./run.bsh
```

## Carla 0.9.15

- [ ] Contient les scripts nécessaires pour carla
- [ ] Configurer et vérifier le fonctionnement de carla 

```
cd carla-0.9.15
sudo ./dependencies.bsh
./install.bsh
sudo python3 -m pip install -r requirements.txt 
```
- [ ] Remplacer /carla-0.9.15/carla/CarlaUE4/Config/DefaultEngine.ini
- [ ] Lancer carla

```
cd carla-0.9.15
sudo ./run.bsh
```

