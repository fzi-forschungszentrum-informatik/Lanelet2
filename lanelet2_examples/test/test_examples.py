import unittest
import os
import subprocess


class Lanelet2ExamplesTestCase(unittest.TestCase):
    def test_run_all_examples(self):
        src = "../src"
        examples = [o for o in os.listdir(src) if os.path.isdir(os.path.join(src, o))]
        devnull = open(os.devnull, 'w')
        for example in examples:
            print("running " + example)
            result = subprocess.call("rosrun lanelet2_examples {}".format(example), shell=True, stdout=devnull)
            self.assertEqual(result, 0, "Failed to execute {}".format(example))

    def test_run_py_examples(self):
        scripts = "../scripts"
        examples = [o for o in os.listdir(scripts) if not os.path.isdir(os.path.join(scripts, o))]
        devnull = open(os.devnull, 'w')
        for example in examples:
            print("running " + example)
            result = subprocess.call("rosrun lanelet2_examples {}".format(example), shell=True, stdout=devnull)
            self.assertEqual(result, 0, "Failed to execute {}".format(example))


if __name__ == '__main__':
    unittest.main()
