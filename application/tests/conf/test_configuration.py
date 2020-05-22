import unittest

from application.conf.configuration import paths, filenames, settings, Initializer, Configuration


class InitializerTest(unittest.TestCase):
    def setUp(self):
        self.list_with_lines = [
            'variable=val', '#Comment = this_is', 'another_var=12',
            'this_is_a bad_line and_next is empty', '', 'bool-value=True',
            'whut=what=with'
        ]
        self.valid_lines_indices = (0, 2, 5)
        self.list_filtered = list(
            tuple(self.list_with_lines[i].split('='))
            for i in self.valid_lines_indices)

    def test_filter_map_settings_empty_list(self):
        self.assertListEqual(Initializer.filter_map_settings(list()), list())

    def test_filter_map_settings_return_list_with_str_tuples(self):
        res = Initializer.filter_map_settings(self.list_with_lines)

        self.assertTrue(isinstance(res, list), 'Did not return a list')
        self.assertTrue(isinstance(res[0], tuple),
                        'List does not contain tuples')
        self.assertEqual(len(res[0]), 2,
                         'Tuple should have 2 elements, variable and value')
        for val in res[0]:
            self.assertTrue(isinstance(val, str),
                            'Element in tuple is not of type str')

    def test_filter_map_settings_list_with_lines(self):
        res = Initializer.filter_map_settings(self.list_with_lines, '#', '=')
        self.assertListEqual(res, self.list_filtered)

    def test_filter_map_settings_list_with_bad_lines(self):
        res = Initializer.filter_map_settings(self.list_with_lines,
                                              comment_sign='#',
                                              assignment_sign='!!')
        self.assertListEqual(res, list())


class ConfigurationTest(unittest.TestCase):

    #def __init__(self, tested_method):
    #    super(ConfigurationTest, self).__init__(tested_method)
    #    print('*****************')
    #    print(tested_method)
    #    if not ConfigurationTest.configuration:
    #        ConfigurationTest.configuration = Configuration(False)

    @classmethod
    def setUpClass(cls):
        '''This class-method in run one time before any test/setups are run'''
        cls.none_instance = Configuration.get_instance(
        )  # Should be None before initialization
        cls.obj = Configuration()

    def setUp(self):
        '''This is run before each test'''
        self.none_instance = ConfigurationTest.none_instance
        self.obj = ConfigurationTest.obj

    def test_configuration_class_get_instance_returns_None_before_init(self):
        self.assertIsNone(self.none_instance)

    def test_configuration_object_get_instance_returns_object(self):
        self.assertIsInstance(self.obj.get_instance(), Configuration)

    def test_configuration_raises_exception_when_initializing_several_objects(
            self):
        with self.assertRaises(RuntimeError) as context_mgr:
            conf = Configuration(False)
        self.assertEqual(
            str(context_mgr.exception),
            'Settings object already created, cannot instantiate a new one. Please use the existing one.'
        )

    def test_configuration_is_created_with_configured_paths(self):
        self.assertDictEqual(self.obj.paths, paths)

    def test_configuration_is_created_with_configured_filenames(self):
        self.assertDictEqual(self.obj.filenames, filenames)

    def test_configuration_is_created_with_configured_settings(self):
        self.assertDictEqual(self.obj.settings, settings)


if __name__ == '__main__':
    unittest.main()
