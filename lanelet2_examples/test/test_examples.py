import unittest
import os
import sys
import subprocess


class Lanelet2ExamplesTestCase(unittest.TestCase):
    def test_run_all_examples(self):
        ros2 = os.environ.get("ROS_VERSION", "") == "2"
        run_cmd = "rosrun lanelet2_examples {}" if not ros2 else "ros2 run lanelet2_examples {}"
        src = "../src"
        examples = [o for o in os.listdir(src) if os.path.isdir(os.path.join(src, o))]
        devnull = open(os.devnull, 'w')
        for example in examples:
            print("running " + example)
            result = subprocess.call(run_cmd.format(example), shell=True)
            self.assertEqual(result, 0, "Failed to execute {}".format(example))

    def test_run_py_examples(self):
        scripts = "../scripts"
        examples = [os.path.join(scripts, o) for o in os.listdir(scripts) if o.endswith(".py")]
        devnull = open(os.devnull, 'w')
        for example in examples:
            print("running " + example)
            result = subprocess.call([sys.executable, example])
            self.assertEqual(result, 0, "Failed to execute {}".format(example))


if __name__ == '__main__':
    unittest.main()
