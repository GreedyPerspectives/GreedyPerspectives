#  Last modified on 1 July 2023
#  Author: Aditya Rauniyar, Krishna Suresh
#  Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
#  Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
#
#  Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.

#  Created on 1 July 2023
#  Author: Aditya Rauniyar, Krishna Suresh
#  Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
#  Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
#
#  Created on 1 7 2023
#  Author: Aditya Rauniyar, Krishna Suresh
#  Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
#  Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
#
#  Created on $file.creation.day $file.creation.month $file.creation.year
#  Author: Aditya Rauniyar, Krishna Suresh
#  Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
#  Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
#
#  Created on 7/1/23 8:18 AM
#  Author: Aditya Rauniyar, Krishna Suresh
#  Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
#  Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
#
#  Created on 1 July 2023
#  Author: Aditya Rauniyar, Krishna Suresh
#  Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
#  Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
#
#  Created on 1 July 2023
#  Author: Aditya Rauniyar, Krishna Suresh
#  Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
#  Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
#
import json
import subprocess


# FIXME: Test this script to install all the dependency from vcpkg.json


def install_dependency(name, version=None):
    dependency = name if version is None else f"{name}:{version}"
    print(f"Installing dependency: {dependency}")
    subprocess.call(["vcpkg", "install", dependency])


def install_dependencies(dependencies):
    for dependency in dependencies:
        if isinstance(dependency, str):
            install_dependency(dependency)
        elif isinstance(dependency, dict):
            name = dependency.get("name")
            version = dependency.get("version")
            if name:
                install_dependency(name, version)
            else:
                print(f"Invalid dependency object: {dependency}")
        else:
            print(f"Invalid dependency format: {dependency}")


def main():
    # Load dependencies from vcpkg.json
    with open('../vcpkg.json') as f:
        vcpkg_data = json.load(f)

    if "dependencies" in vcpkg_data:
        dependencies = vcpkg_data["dependencies"]
        install_dependencies(dependencies)
    else:
        print("No dependencies found in vcpkg.json.")


if __name__ == "__main__":
    main()
