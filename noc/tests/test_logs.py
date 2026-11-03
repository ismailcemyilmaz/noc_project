import os
import tempfile
import unittest

import numpy as np

from noc.analysis.logs import read_log


class TestReadLog(unittest.TestCase):
    def _write(self, text):
        fd, p = tempfile.mkstemp(suffix='.log')
        with os.fdopen(fd, 'w') as f:
            f.write(text)
        self.addCleanup(os.remove, p)
        return p

    def test_header_only_is_empty(self):
        p = self._write('ts efx efy\n')
        lg = read_log(p)
        self.assertEqual(lg.n, 0)
        self.assertEqual(lg['efx'].shape, (0,))

    def test_comments_placeholders_and_bad_rows(self):
        p = self._write('# comment\n#\nts a b\n1.0 2 -\n2.0 nan 3\n3.0 1\n')
        lg = read_log(p)
        self.assertEqual(lg.n, 2)
        self.assertEqual(lg.n_bad, 1)
        self.assertTrue(np.isnan(lg['b'][0]))
        self.assertTrue(np.isnan(lg['a'][1]))
        np.testing.assert_allclose(lg['ts'], [1.0, 2.0])

    def test_text_column_dropped(self):
        p = self._write('name ts x\nHR_6 1.5 0.25\n')
        lg = read_log(p, text_cols=('name',))
        self.assertEqual(lg.header, ['ts', 'x'])
        self.assertAlmostEqual(lg['x'][0], 0.25)


if __name__ == '__main__':
    unittest.main()
