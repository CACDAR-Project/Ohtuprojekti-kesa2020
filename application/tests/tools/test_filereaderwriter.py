import unittest
from unittest.mock import patch, mock_open
from io import StringIO
from contextlib import redirect_stdout

from application.tools.filereaderwriter import check_directory_exists, check_file_exists, read_file_lines, create_directory


class FilereaderwriterTest(unittest.TestCase):
    def setUp(self):
        self.file_contents_mock = '\n'.join(
            ('# This is a mock of the settings file.', 'var=val',
             'something =smthing',
             'A_VERY_LONG_VARIABLE= with value with value and whitespaces  ',
             '#Some people like commenting', 'end-test=True'))
        self.mock_directory_path = 'config'
        self.mock_settings_file_path = 'config/settings'

        # In-memory mock of file
        self.file_mock = mock_open(read_data=self.file_contents_mock)

    @patch('os.path.isdir')
    def test_check_directory_exists_false(self, mock_isdir):
        mock_isdir.return_value = False
        self.assertFalse(check_directory_exists(self.mock_directory_path))

    @patch('os.path.isdir')
    def test_check_directory_exists_true(self, mock_isdir):
        mock_isdir.return_value = True
        self.assertTrue(check_directory_exists(self.mock_directory_path))

    @patch('os.path.isfile')
    def test_check_file_exists_false(self, mock_isfile):
        mock_isfile.return_value = False
        self.assertFalse(check_file_exists(self.mock_settings_file_path))

    @patch('os.path.isfile')
    def test_check_file_exists_true(self, mock_isfile):
        mock_isfile.return_value = True
        self.assertTrue(check_file_exists(self.mock_settings_file_path))

    @patch('os.path.isfile')
    def test_read_file_lines_file_not_found(self, mock_isfile):
        mock_isfile.return_value = False
        res = None

        # Redirect stdout error message to StringIO to keep test output clean.
        # Also assert that we get an error message.
        with redirect_stdout(StringIO()) as stdout:
            with patch('builtins.open', new=self.file_mock) as f:
                res = read_file_lines(self.mock_settings_file_path)
                f.assert_not_called()  # Assert that file has not been opened

        self.assertEqual(
            stdout.getvalue(),
            f'Error while trying to read file: \'{self.mock_settings_file_path}\'.\n'
        )
        self.assertListEqual(res, [])

    @patch('os.path.isfile')
    def test_read_file_lines(self, mock_isfile):
        mock_isfile.return_value = True
        res_lines = None

        with patch('builtins.open', new=self.file_mock) as f:
            res_lines = read_file_lines(self.mock_settings_file_path)
            f.assert_called_once_with(self.mock_settings_file_path, 'r')

        self.assertListEqual(res_lines, self.file_contents_mock.split('\n'))

    @patch('os.makedirs')
    def test_makedir_success(self, mock_makedirs):
        create_directory(self.mock_directory_path)
        mock_makedirs.assert_called_once_with(self.mock_directory_path,
                                              exist_ok=True)


# Now this is the actual case that would be interesting to test, not so easy though..
#    @patch('os.makedirs')
#    def test_makedir_failure(self, mock_makedirs):
#        # testing tests..
#        #mock_makedirs.side_effects = OSError('Error was thrown')
#        mock_makedirs.return_value = 0
#        #

#        with redirect_stdout(StringIO()) as stdout:
#            #with self.assertRaises(OSError):
#            create_directory(self.mock_directory_path)
#        print('ERRRRRROR', stdout.getvalue())
#        self.assertEqual(stdout.getvalue(), 'testingtesting')
#        #    '\n'.join[
#        #        'Error while trying to create directory \'{self.mock_directory_path}\'.',
#        #        'exception error output here',
#        #        'Exiting..\n'])


if __name__ == '__main__':
    unittest.main()
