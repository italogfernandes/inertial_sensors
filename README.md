# Inertial Sensors
Ferramentas e implementações úteis para o trabalho com sensores inerciais.

## Getting Started

Implementação de Algoritimos para calculo de orientação, assim como outras
ferramentas para o estudo de sensores inerciais.

![](/Docs/Pictures/interface.gif)

### In this repository

There are a lot of folders and files, but they are mostly empty :( - we need to get to work.

- Docs: Imagens, guias, referencias, codigos consultados.
- Arduino: Codigos para leitura de sensores inerciais.
  - [x] mpu6050_dataReader.
  - [ ] mpu6050_fakedataReader. **Last Modified**
- [ ] Unity: Interface para visualização das orientações. **Working on it.**
- [ ] Python: Alguns testes do algoritimo de madgwick em python.

## Development Environment

* Arduino:
  *  [Platomformio](https://atom.io/packages/platomformio) - Atom integration with PlatformIO (for building arduino files,but arduino IDE can also be used.).
  * [Arduino IDE](www.arduino.cc) - If you prefer.
  * [Library for Inertial Sensor](https://github.com/jrowberg/i2cdevlib) - Download and place the folders Arduino/MPU6050, Arduino/HMC5883L and Arduino/i2cdevlib in the right place.
* Unity 5.6.3xf1 (Preferred)- [Linux Instructions](https://forum.unity3d.com/threads/unity-on-linux-release-notes-and-known-issues.350256/) - [Windows and Mac](https://unity3d.com/).
* [Python2.7](https://www.python.org/downloads/).


## More Info

* [Artigo do Madgwick](http://x-io.co.uk/res/doc/madgwick_internal_report.pdf).
* [Gait Tracking](https://github.com/italogfernandes/Gait-Tracking-With-x-IMU).

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

```
"THE BEERWARE LICENSE" (Revision 42):
Italo Fernandes and Andrei Nakagawa wrote this code.
As long as you retain this notice, you can do whatever you want with this stuff.
If we meet someday, and you think this stuff is worth it, you can buy us a beer in return.
```
## Authors

* **Italo Fernandes** - https://github.com/italogfernandes - italogsfernandes@gmail.com

* **Andrei Nakagawa** - https://github.com/andreinakagawa - www.biolab.eletrica.ufu.br

See also the list of [contributors](https://github.com/italogfernandes/inertial-sensors/contributors) who participated in this project.
