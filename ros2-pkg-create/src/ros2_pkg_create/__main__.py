import copier


def main():

    copier.run_copy("https://gitlab.ika.rwth-aachen.de/fb-fi/ops/templates/ros2/package_generator.git", ".", unsafe=True, vcs_ref="pipeline")


if __name__ == "__main__":
    main()
