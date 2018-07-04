"""A class that handles writing out matrices of a system to a C++ or Java file.
"""

import numpy as np
import os


class SystemWriter:

    def __init__(self, system, system_name, period_variant=False):
        """Exports matrices to pair of C++ source files.

        Keyword arguments:
        system -- System object
        system_name -- subsystem class name in camel case
        period_variant -- True to use PeriodVariantLoop, False to use
                          StateSpaceLoop
        """
        self.system = system
        self.system_name = system_name
        self.template = "<" + str(system.sysd.A.shape[0]) + ", " + str(
            system.sysd.B.shape[1]) + ", " + str(system.sysd.C.shape[0]) + ">"

        self.period_variant = period_variant
        if period_variant:
            self.plant_coeffs = "PeriodVariantPlantCoeffs"
            self.observer_coeffs = "PeriodVariantKalmanFilterCoeffs"
        else:
            self.plant_coeffs = "StateSpacePlantCoeffs"
            self.observer_coeffs = "StateSpaceObserverCoeffs"
        self.controller_coeffs = "StateSpaceControllerCoeffs"

    def write_cpp_header(self):
        """Writes C++ header file."""
        prefix = "#include \"Controllers/"
        headers = []
        headers.append(prefix + self.plant_coeffs + ".h\"")
        headers.append(prefix + self.controller_coeffs + ".h\"")
        headers.append(prefix + self.observer_coeffs + ".h\"")

        plant_coeffs_type = "frc::" + self.plant_coeffs
        controller_coeffs_type = "frc::" + self.controller_coeffs
        observer_coeffs_type = "frc::" + self.observer_coeffs

        with open(self.system_name + "Coeffs.h", "w") as header_file:
            header_file.write("#pragma once" + os.linesep + os.linesep)
            for header in sorted(headers):
                header_file.write(header + os.linesep)
            header_file.write(os.linesep)
            header_file.write(plant_coeffs_type + self.template + " Make" +
                              self.system_name + "PlantCoeffs();" + os.linesep)
            header_file.write(controller_coeffs_type + self.template + " Make" +
                              self.system_name + "ControllerCoeffs();" +
                              os.linesep)
            header_file.write(observer_coeffs_type + self.template + " Make" +
                              self.system_name + "ObserverCoeffs();" +
                              os.linesep)

    def write_cpp_source(self, header_path_prefix):
        """Writes C++ source file.

        Keyword arguments:
        header_prefix -- path prefix in which header exists
        """
        plant_coeffs_type = "frc::" + self.plant_coeffs
        controller_coeffs_type = "frc::" + self.controller_coeffs
        observer_coeffs_type = "frc::" + self.observer_coeffs

        with open(self.system_name + "Coeffs.cpp", "w") as source_file:
            source_file.write("#include \"" + header_path_prefix +
                              self.system_name + "Coeffs.h\"" + os.linesep +
                              os.linesep)
            source_file.write("#include <Eigen/Core>" + os.linesep + os.linesep)

            source_file.write(plant_coeffs_type + self.template + " Make" +
                              self.system_name + "PlantCoeffs() {" + os.linesep)
            if self.period_variant:
                self.__write_cpp_matrix(source_file, self.system.sysc.A,
                                        "Acontinuous")
                self.__write_cpp_matrix(source_file, self.system.sysc.B,
                                        "Bcontinuous")
                self.__write_cpp_matrix(source_file, self.system.sysd.C, "C")
                self.__write_cpp_matrix(source_file, self.system.sysd.D, "D")
                source_file.write(
                    "  return " + plant_coeffs_type + self.template +
                    "(Acontinuous, Bcontinuous, C, D);" + os.linesep)
                source_file.write("}" + os.linesep + os.linesep)
            else:
                self.__write_cpp_matrix(source_file, self.system.sysd.A, "A")
                self.__write_cpp_matrix(source_file,
                                        np.linalg.inv(self.system.sysd.A),
                                        "Ainv")
                self.__write_cpp_matrix(source_file, self.system.sysd.B, "B")
                self.__write_cpp_matrix(source_file, self.system.sysd.C, "C")
                self.__write_cpp_matrix(source_file, self.system.sysd.D, "D")
                source_file.write("  return " + plant_coeffs_type +
                                  self.template + "(A, Ainv, B, C, D);" +
                                  os.linesep)
                source_file.write("}" + os.linesep + os.linesep)

            source_file.write(controller_coeffs_type + self.template + " Make" +
                              self.system_name + "ControllerCoeffs() {" +
                              os.linesep)
            self.__write_cpp_matrix(source_file, self.system.K, "K")
            self.__write_cpp_matrix(source_file, self.system.Kff, "Kff")
            self.__write_cpp_matrix(source_file, self.system.u_min, "Umin")
            self.__write_cpp_matrix(source_file, self.system.u_max, "Umax")
            source_file.write("  return " + controller_coeffs_type +
                              self.template + "(K, Kff, Umin, Umax);" +
                              os.linesep)
            source_file.write("}" + os.linesep + os.linesep)

            source_file.write(plant_coeffs_type + self.template + " Make" +
                              self.system_name + "ObserverCoeffs() {" +
                              os.linesep)
            if self.period_variant:
                self.__write_cpp_matrix(source_file, self.system.Q,
                                        "Qcontinuous")
                self.__write_cpp_matrix(source_file, self.system.R,
                                        "Rcontinuous")
                self.__write_cpp_matrix(source_file, self.system.P_steady,
                                        "PsteadyState")
                source_file.write("  return " + observer_coeffs_type +
                                  self.template + "(Qcontinuous, Rcontinuous," +
                                  os.linesep +
                                  " " * len("  return " + observer_coeffs_type +
                                            self.template + "(") +
                                  "PsteadyState);" + os.linesep)
                source_file.write("}" + os.linesep)
            else:
                self.__write_cpp_matrix(source_file, self.system.L, "L")
                source_file.write("  return " + observer_coeffs_type +
                                  self.template + "(L);" + os.linesep)
                source_file.write("}" + os.linesep)

    def __write_cpp_matrix(self, cpp_file, matrix, matrix_name):
        cpp_file.write("  Eigen::Matrix<double, " + str(matrix.shape[0]) +
                       ", " + str(matrix.shape[1]) + "> " + matrix_name + ";" +
                       os.linesep)
        for row in range(matrix.shape[0]):
            for col in range(matrix.shape[1]):
                cpp_file.write("  " + matrix_name + "(" + str(row) + ", " +
                               str(col) + ") = " + str(matrix[row, col]) + ";" +
                               os.linesep)
