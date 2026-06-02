"""
tests/test_memory_sqlite.py
===========================
Verifies that SQLite memory database is successfully created,
can store facts/conversations/experiences, and returns matched term queries.
"""

import sys
import unittest
from pathlib import Path

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from ai.memory.memory_manager import MemoryManager

class TestMemorySQLite(unittest.TestCase):
    def setUp(self):
        # Initialize an in-memory database or temporary database for testing
        self.mgr = MemoryManager(db_path=":memory:")

    def test_store_and_retrieve_preference(self):
        # Store preferences
        self.mgr.store_preference("user_nickname", "Boss Adi")
        self.mgr.store_preference("garage_alias", "the cave")
        
        # Retrieve context matching terms
        context = self.mgr.retrieve_context("What does Adi call the garage?")
        
        # Check assertions
        self.assertIn("the cave", context)
        self.assertIn("garage_alias", context)

    def test_store_and_retrieve_conversation(self):
        # Store conversation
        self.mgr.store_conversation("user", "We mapped the living room.")
        self.mgr.store_conversation("archer", "Affirmative. Storing mapped coordinates.")
        
        # Retrieve context
        context = self.mgr.retrieve_context("mapped room")
        self.assertIn("We mapped the living room.", context)
        self.assertIn("archer:", context)

    def test_store_and_retrieve_experience(self):
        # Store experience
        self.mgr.store_experience("nav_success", "Successfully navigated to the kitchen.")
        
        # Retrieve context
        context = self.mgr.retrieve_context("kitchen")
        self.assertIn("Successfully navigated to the kitchen.", context)
        self.assertIn("nav_success", context)

    def test_locations_store(self):
        self.mgr.store_location("kitchen", 2.0, -6.5, 0.0, "kitchen")
        context = self.mgr.retrieve_context("where is the kitchen?")
        self.assertIn("kitchen", context)
        self.assertIn("2.0", context)
        self.assertIn("-6.5", context)

if __name__ == "__main__":
    unittest.main()
