# Devro #

## Description ##
Development tools and utilities for SLAM, including an environment to test your own algorithms, as well as readily usable algorithms for SLAM, path planning and motion planning.

<img src="https://raw.githubusercontent.com/abnvar/devro/master/res/envWithDynamicObstacles.gif" />

## Usage ##

```bash
$ pip install devro
```

or install from source

```bash
$ git clone https://github.com/abnvar/devro/
$ cd devro
$ python3 -m pip install -r requirements.txt
$ python3 -m pip install -e .
```

## Features ##

1. __Environment__
    1. Random obstacle generation using __Perlin noise__

          <kbd>
            <img src="https://raw.githubusercontent.com/abnvar/devro/master/res/randomMaps/map0.png" />
          </kbd>
          <kbd>
            <img src="https://raw.githubusercontent.com/abnvar/devro/master/res/randomMaps/map1.png" />
          </kbd>
          <kbd>
            <img src="https://raw.githubusercontent.com/abnvar/devro/master/res/randomMaps/map2.png" />
          </kbd>

    2. Dynamic Obstacles

          You can attach dynamic obstacles to the environment . The basic API is extremely simple , You just need to call  __addDynamicObstacles()__ on simulator class . This function accepts three parameter :

          a. Quantity ``` qty``` : Total number of dynamic obstacles

          b. Size ```radiusRange``` : Minimum and Maximum radius of obstacles
          
          c. Maximum Velocity    ```maxVelocity``` : Maximum Velocity obstacles can attend .

          Usage:

          ```python
          sim.addDynamicObstacles(qty=3, radiusRange=(10,20), maxVelocity=10)

          ```



    3. API-like interaction.

          ```python
          lidar = Lidar(*args)                      #
          bot = Bot(*args)                          # initialization
          sim = Simulation(*args)                   #

          sim.begin()                               # start sim

          bot.setVel(leftVelocity, rightVelocity)   # set wheel velocities
          bot.scan()                                # lidar scan
          ```

2. __Algorithms__
    1. __SLAM__

          Desciption

    2. __Path Planning / Trajectory Planning__

          Desciption

    3. __Motion Planning__

          Desciption


## Collaborators ##

- [brunopinto900](https://github.com/brunopinto900)
- [santoshdahal2016](https://github.com/santoshdahal2016)
- [ObjectionTheory](https://github.com/ObjectionTheory)

## For Developers ##

1. __Threading__
   - __Window__
     - Thread runs the tkinter display.
     - __Interaction__:
       - Start: Simulation(visualize = True)
       - Exit: Daemon thread. Exits with end of the main thread.
   - __Simulation__
     - Thread runs the simpy simulation.
     - __Interaction__:
       - Start: Simulation()
       - Exit: Daemon thread. Exits with end of the main thread.
