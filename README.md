# Devro #

## Description ##
Development tools and utilities for SLAM, including an environment to test your own algorithms, as well as readily usable algorithms for SLAM, path planning and motion planning.

![Environment](https://raw.githubusercontent.com/abnvar/devro/master/res/env.gif)

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

    2. API-like interaction.

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
- [sinha5432](https://github.com/sinha5432)
