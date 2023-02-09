I prefer to use a virtual environment for python. My method is to use virtualenvwrapper. For example:
```shell
sudo apt install -y virtualenvwrapper
source /usr/share/virtualenvwrapper/virtualenvwrapper.sh  # can add to ~/.bashrc
mkvirtualenv env1
add2virtualenv src
cd build
cmake -DBUILD_PYTHON_API=1 ..
make -j
```

To switch python versions in the build folder I just reset things:
```shell
rm CMakeCache.txt
```

In order to use the `python` virtual environment in a new terminal:
```shell
workon env1
```

An example of running in an interactive `python` session:
```python
import motor
m = motor.MotorManager()
m.set_command_mode(motor.ModeDesired.Position)
m.set_command_position([0])
m.write_saved_commands()
m.set_command_position([10])
m.write_saved_commands()
```
