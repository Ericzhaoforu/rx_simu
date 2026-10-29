import unittest
from usv_nav2.task_lease import TaskLease


class LeaseTests(unittest.TestCase):
    def test_fresh_resume_clears_cached_command(self):
        lease=TaskLease();self.assertFalse(lease.valid(10,10))
        self.assertTrue(lease.update(1,True,10,10,10));self.assertTrue(lease.valid(10.1,10.1))
        self.assertFalse(lease.update(1,True,10.1,10.1,10.1))
        self.assertTrue(lease.update(2,False,10.2,10.2,10.2));self.assertFalse(lease.valid(10.2,10.2))
        self.assertTrue(lease.update(2,True,10.3,10.3,10.3))

    def test_stale_reordered_or_frozen_clock_never_renews(self):
        lease=TaskLease();lease.update(2,True,10,10,10)
        self.assertFalse(lease.update(1,True,10.1,10.1,10.1))
        self.assertFalse(lease.update(2,True,10,10.2,10.2))
        self.assertFalse(lease.valid(10,10.6))
        self.assertFalse(lease.valid(10.6,10.1))
        self.assertTrue(lease.update(2,True,10.7,10.7,10.7))


if __name__=='__main__':unittest.main()
