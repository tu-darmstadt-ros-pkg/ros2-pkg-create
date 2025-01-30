import argcomplete
import argparse
import os
import copier
from ament_index_python.packages import get_package_share_directory

try:
    import git
except ImportError:
    print(
        "GitPython is required! Install using 'pip3 install --user gitpython' or 'apt install python3-git'"
    )
    raise

try:
    import copier
except ImportError:
    print(
        "GitPython is required! Install using 'pip3 install --user copier'"
    )
    raise


def parseArguments() -> argparse.Namespace:

    parser = argparse.ArgumentParser(description="Creates a ROS 2 package from templates")

    parser.add_argument("destination", type=str, help="Destination directory")
    parser.add_argument("--defaults", action="store_true", help="Use defaults for all options")
    parser.add_argument("--use-local-templates", action="store_true", help="Use locally installed templates instead of remotely pulling most recent ones")

    # parser.add_argument("--template", type=str, default=None, choices=["cpp_pkg", "msgs_pkg", "python_pkg", "ci"], required=True, help="Template")
    parser.add_argument("--package-name", type=str, default=None, help="Package name")
    parser.add_argument("--description", type=str, default=None, help="Description")
    parser.add_argument("--maintainer", type=str, default=None, help="Maintainer")
    parser.add_argument("--maintainer-email", type=str, default=None, help="Maintainer email")
    parser.add_argument("--author", type=str, default=None, help="Author")
    parser.add_argument("--author-email", type=str, default=None, help="Author email")
    parser.add_argument("--license", type=str, default=None, choices=["Apache-2.0", "BSL-1.0", "BSD-2.0", "BSD-2-Clause", "BSD-3-Clause", "GPL-3.0-only", "LGPL-2.1-only", "LGPL-3.0-only", "MIT", "MIT-0"], help="License")
    parser.add_argument("--node-name", type=str, default=None, help="Node name")
    parser.add_argument("--node-class-name", type=str, default=None, help="Class name of node")
    parser.add_argument("--is-component", action="store_true", default=None, help="Make it a component?")
    parser.add_argument("--no-is-component", dest="is-component", default=None, action="store_false")
    parser.add_argument("--is-lifecycle", action="store_true", default=None, help="Make it a lifecycle node?")
    parser.add_argument("--no-is-lifecycle", dest="is-lifecycle", default=None, action="store_false")
    parser.add_argument("--has-launch-file", action="store_true", default=None, help="Add a launch file?")
    parser.add_argument("--no-has-launch-file", dest="has-launch-file", default=None, action="store_false")
    parser.add_argument("--launch-file-type", type=str, choices=["xml", "py", "yml"], help="Type of launch file")
    parser.add_argument("--has-params", action="store_true", default=None, help="Add parameter loading")
    parser.add_argument("--no-has-params", dest="has-params", default=None, action="store_false")
    parser.add_argument("--has-subscriber", action="store_true", default=None, help="Add a subscriber?")
    parser.add_argument("--no-has-subscriber", dest="has-subscriber", default=None, action="store_false")
    parser.add_argument("--has-publisher", action="store_true", default=None, help="Add a publisher?")
    parser.add_argument("--no-has-publisher", dest="has-publisher", default=None, action="store_false")
    parser.add_argument("--has-service-server", action="store_true", default=None, help="Add a service server?")
    parser.add_argument("--no-has-service-server", dest="has-service-server", default=None, action="store_false")
    parser.add_argument("--has-action-server", action="store_true", default=None, help="Add an action server?")
    parser.add_argument("--no-has-action-server", dest="has-action-server", default=None, action="store_false")
    parser.add_argument("--has-timer", action="store_true", default=None, help="Add a timer callback?")
    parser.add_argument("--no-has-timer", dest="has-timer", default=None, action="store_false")
    parser.add_argument("--auto-shutdown", action="store_true", default=None, help="Automatically shutdown the node after launch (useful in CI/CD)?")
    parser.add_argument("--no-auto-shutdown", dest="auto-shutdown", default=None, action="store_false")
    parser.add_argument("--interface-types", type=str, default=None, choices=["Message", "Service", "Action"], help="Interfaces types")
    parser.add_argument("--msg-name", type=str, default=None, help="Message name")
    parser.add_argument("--srv-name", type=str, default=None, help="Service name")
    parser.add_argument("--action-name", type=str, default=None, help="Action name")


    argcomplete.autocomplete(parser)
    return parser.parse_args()

def add_git_config_info(answers):
    # add author and maintainer info from git config if not yet set
    git_config = git.GitConfigParser()
    git_config.read()
    if not answers.get("author") or not answers.get("maintainer"):
        answers["user_name_git"] = git_config.get_value("user", "name")
    if not answers.get("author_email") or not answers.get("maintainer_email"):
        answers["user_email_git"] = git_config.get_value("user", "email")

def create():

    # pass specified arguments as data to copier
    args = parseArguments()
    answers = {k: v for k, v in vars(args).items() if v is not None}

    # add author and maintainer info from git config if not yet set
    add_git_config_info(answers)

    # get pkg template location if installed as ros pkg
    package_name = 'ros2-pkg-create'
    try:
        template_location = get_package_share_directory(package_name)
        print(f"The share directory for '{package_name}' is: {template_location}")
    except KeyError:
        print(f"Package '{package_name}' not found in the ament index.")
        if not __file__.startswith("/opt"):
            print(f"Trying to use local templates in ")    
            template_location = os.path.join(os.path.dirname(__file__), "..")

    # run copier
    try:
        copier.run_copy(template_location,
                        os.path.join(os.getcwd(), args.destination),
                        data=answers,
                        defaults=args.defaults,
                        unsafe=True,
                        vcs_ref="HEAD")
                        
    except copier.CopierAnswersInterrupt:
        print("Aborted")
        return

if __name__ == "__main__":
    create()
