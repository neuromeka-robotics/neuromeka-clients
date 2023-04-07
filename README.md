# Neuromeka Clients

This package provides client protocols for users to interact with Neuromeka's products, including Indy, Moby, Ecat, and Motor.

## Installation

You can install the package from PyPI:

```bash
pip install neuromeka-clients
```

## Usage
The package contatins the following client classes:

* IndyClient in indy.py
* MobyClient in moby.py
* EcatClient in ecat.py
* MotorClient in motor.py

To use a client class, simply import it and create an instance:

```python
from neuromeka import EcatClient, MobyClient, IndyClient, MotorClient

ecat = EcatClient("192.168.214.20")
moby = IndyClient("192.168.214.20")
indy = IndyClient("192.168.0.11")
motor = MotorClient("192.168.0.20")
```

Replace EcatClient with the desired client class and the IP address with the appropriate address for your device.

## Dependencies
This package requires the following dependencies:

* grpcio
* grpcio-tools
* protobuf

These dependencies will be automatically installed when you install the package using pip.

## Examples
Please refer to the 'example.py' file in the package for usage examples.

## Support
If you encounter any issues or need help, please open an issue on the project's repository.

## License
This package is released under the [LICENSE_NAME]. For more information, please refer to the LICENSE file.



