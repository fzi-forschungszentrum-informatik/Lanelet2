from setuptools import setup

DISTNAME = "lanelet2"
DESCRIPTION = "Map handling framework for automated driving."
MAINTAINER = "Fabian Immel"
MAINTAINER_EMAIL = "fabian.immel@kit.edu"
URL = "https://github.com/fzi-forschungszentrum-informatik/Lanelet2"
LICENSE = "BSD"
DOWNLOAD_URL = ""
VERSION = '1.2.0'
PYTHON_VERSION = (3, 8, 10)

class ExtModules(list):
    def __bool__(self):
        return True

setup(name=DISTNAME,
      description=DESCRIPTION,
      maintainer=MAINTAINER,
      maintainer_email=MAINTAINER_EMAIL,
      url=URL,
      license=LICENSE,
      download_url=DOWNLOAD_URL,
      version=VERSION,
      packages=["lanelet2"],
      # Include pre-compiled extension
      package_data={"lanelet2": ["*"]},
      ext_modules=ExtModules(),
      has_ext_modules=lambda: True)
