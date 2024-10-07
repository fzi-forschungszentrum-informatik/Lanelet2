from conans import ConanFile
from conan.errors import ConanException
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import os


# pylint: disable=W0201
class PythonDevConfigConan(ConanFile):
    name = "python_dev_config"
    version = "0.6"
    license = "MIT"
    description = "Configuration of Python interpreter for use as a development dependency."
    url = "https://github.com/bincrafters/conan-python_dev_config"
    options = {"python": "ANY"}
    default_options = {'python': 'python'}
    settings = "os", "arch", "compiler"
    build_policy = "missing"

    def export(self):
        self.copy("LICENSE.md")

    def package_id(self):
        self.info.header_only()
        self.info.options.python_version = self._python_version

    def package_info(self):
        if self.have_python_dev:
            self.cpp_info.includedirs = [self._python_include]
            self.cpp_info.libdirs = [os.path.dirname(self._python_lib)]
            self.cpp_info.libs = [self._python_lib_ldname]
            self.cpp_info.bindirs = [os.path.dirname(self._python_lib), os.path.dirname(self._python_exec)]
            self.user_info.python_version = self._python_version
            self.user_info.python_exec = self._python_exec
            self.user_info.python_include_dir = self._python_include
            self.user_info.python_lib = self._python_lib
            self.user_info.python_lib_dir = os.path.dirname(self._python_lib)
            self.env_info.path.append(os.path.dirname(self._python_lib))
            self.env_info.path.append(os.path.dirname(self._python_exec))

    @property
    def have_python_dev(self):
        if not self._python_exec:
            return False
        if not self._python_include:
            return False
        if not self._python_lib:
            return False
        if not os.path.exists(os.path.join(self._python_include, 'Python.h')):
            return False
        return True

    @property
    def _is_msvc(self):
        return self.settings.compiler == "Visual Studio"

    @property
    def _python_exec(self):
        """
        obtain full path to the python interpreter executable
        :return: path to the python interpreter executable, either set by option, or system default
        """
        if not hasattr(self, '_py_exec'):
            self._py_exec = self._run_python_script("from __future__ import print_function; "
                                                    "import sys; "
                                                    "print(sys.executable)",
                                                    str(self.options.python))
        return self._py_exec

    @property
    def _python_version(self):
        """
        obtain version of python interpreter
        :return: python interpreter version, in format major.minor
        """
        if not hasattr(self, '_py_version'):
            self._py_version = self._run_python_script("from __future__ import print_function; "
                                                       "import sys; "
                                                       "print('%s.%s' % (sys.version_info[0], sys.version_info[1]))")
        return self._py_version

    @property
    def _python_lib(self):
        """
        attempt to find python development library
        :return: the full path to the python library to be linked with
        """
        if not hasattr(self, '_py_lib'):
            self._py_lib = None
            library = self._get_python_var("LIBRARY")
            ldlibrary = self._get_python_var("LDLIBRARY")
            libdir = self._get_python_var("LIBDIR")
            multiarch = self._get_python_var("MULTIARCH")
            masd = self._get_python_var("multiarchsubdir")
            with_dyld = self._get_python_var("WITH_DYLD")
            if libdir and multiarch and masd:
                if masd.startswith(os.sep):
                    masd = masd[len(os.sep):]
                libdir = os.path.join(libdir, masd)

            if not libdir:
                libdest = self._get_python_var("LIBDEST")
                libdir = os.path.join(os.path.dirname(libdest), "libs")

            candidates = [ldlibrary, library]
            library_prefixes = [""] if self._is_msvc else ["", "lib"]
            library_suffixes = [".lib"] if self._is_msvc else [".so", ".dll.a", ".a"]
            if with_dyld:
                library_suffixes.insert(0, ".dylib")

            python_version = self._python_version
            python_version_no_dot = python_version.replace(".", "")
            versions = ["", python_version, python_version_no_dot]
            abiflags = self._python_abiflags

            for prefix in library_prefixes:
                for suffix in library_suffixes:
                    for version in versions:
                        candidates.append("%spython%s%s%s" % (prefix, version, abiflags, suffix))

            for candidate in candidates:
                if candidate:
                    python_lib = os.path.join(libdir, candidate)
                    self.output.info('checking %s' % python_lib)
                    if os.path.isfile(python_lib):
                        self.output.info('found python library: %s' % python_lib)
                        self._py_lib = python_lib.replace('\\', '/')
                        return self._py_lib
        return self._py_lib
    
    @property
    def _python_lib_ldname(self):
        if not hasattr(self, '_py_lib_ldname'):
            libname = os.path.splitext(os.path.basename(self._python_lib))[0]
            if not self._is_msvc:
                libname = libname[3:] if libname.startswith("lib") else libname
            self._py_lib_ldname = libname
        return self._py_lib_ldname

    @property
    def _python_bindir(self):
        if not hasattr(self, '_py_bindir'):
            self._py_bindir = self._get_python_var('BINDIR')
        return self._py_bindir

    @property
    def _python_include(self):
        """
        attempt to find directory containing Python.h header file
        :return: the directory with python includes
        """
        if not hasattr(self, '_py_include'):
            self._py_include = None
            include = self._get_python_path('include')
            plat_include = self._get_python_path('platinclude')
            include_py = self._get_python_var('INCLUDEPY')
            include_dir = self._get_python_var('INCLUDEDIR')
            python_inc = self._python_inc

            candidates = [include,
                          plat_include,
                          include_py,
                          include_dir,
                          python_inc]
            for candidate in candidates:
                if candidate:
                    python_h = os.path.join(candidate, 'Python.h')
                    self.output.info('checking %s' % python_h)
                    if os.path.isfile(python_h):
                        self.output.info('found Python.h: %s' % python_h)
                        self._py_include = candidate.replace('\\', '/')
                        return self._py_include
        return self._py_include

    @property
    def _python_inc(self):
        """
        obtain the result of the "sysconfig.get_python_inc()" call
        :return: result of the "sysconfig.get_python_inc()" execution
        """
        return self._run_python_script("from __future__ import print_function; "
                                       "import sysconfig; "
                                       "print(sysconfig.get_python_inc())")

    def _get_python_sc_var(self, name):
        """
        obtain value of python sysconfig variable
        :param name: name of variable to be queried (such as LIBRARY or LDLIBRARY)
        :return: value of python sysconfig variable
        """
        return self._run_python_script("from __future__ import print_function; "
                                       "import sysconfig; "
                                       "print(sysconfig.get_config_var('%s'))" % name)

    def _get_python_du_var(self, name):
        """
        obtain value of python distutils sysconfig variable
        (sometimes sysconfig returns empty values, while python.sysconfig provides correct values)
        :param name: name of variable to be queried (such as LIBRARY or LDLIBRARY)
        :return: value of python sysconfig variable
        """
        return self._run_python_script("from __future__ import print_function; "
                                       "import distutils.sysconfig as du_sysconfig; "
                                       "print(du_sysconfig.get_config_var('%s'))" % name)

    def _get_python_var(self, name):
        """
        obtain value of python variable, either by sysconfig, or by distutils.sysconfig
        :param name: name of variable to be queried (such as LIBRARY or LDLIBRARY)
        :return: value of python sysconfig variable
        """
        return self._get_python_sc_var(name) or self._get_python_du_var(name)

    def _get_python_path(self, name):
        """
        obtain path entry for the python installation
        :param name: name of the python config entry for path to be queried (such as "include", "platinclude", etc.)
        :return: path entry from the sysconfig
        """
        # https://docs.python.org/3/library/sysconfig.html
        # https://docs.python.org/2.7/library/sysconfig.html
        return self._run_python_script("from __future__ import print_function; "
                                       "import sysconfig; "
                                       "print(sysconfig.get_path('%s'))" % name)

    @property
    def _python_abiflags(self):
        """
        obtain python ABI flags, see https://www.python.org/dev/peps/pep-3149/ for the details
        :return: the value of python ABI flags
        """
        return self._run_python_script("from __future__ import print_function; "
                                       "import sys; "
                                       "print(getattr(sys, 'abiflags', ''))")

    def _run_python_script(self, script, python_exec=None):
        """
        execute python one-liner script and return its output
        :param script: string containing python script to be executed
        :return: output of the python script execution, or None, if script has failed
        """
        python_exec = python_exec or self._python_exec
        output = StringIO()
        command = '"%s" -c "%s"' % (python_exec, script)
        self.output.info('running %s' % command)
        try:
            self.run(command=command, output=output)
        except ConanException:
            self.output.info("(failed)")
            return None
        output = output.getvalue().strip()
        self.output.info(output)
        return output if output != "None" else None
