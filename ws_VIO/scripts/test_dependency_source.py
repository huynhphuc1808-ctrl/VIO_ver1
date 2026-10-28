"""Exercise dependency integrity checks on standalone source archives."""

import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from verify_dependency_source import verify_source


class DependencySourceTests(unittest.TestCase):
    """Check source corruption and archive/monorepo behavior before builds."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "dep"
        self.source.mkdir()
        self.file = self.source / "sample.cpp"
        self.file.write_text("int value = 1;\n")
        self.pin = "a" * 40
        self.lock = self.root / "DEPENDENCIES.lock.json"
        self.lock.write_text(json.dumps({
            "schema_version": 1,
            "dependencies": {"dep": {
                "commit": self.pin,
                "files": {"sample.cpp": {
                    "sha256": hashlib.sha256(self.file.read_bytes()).hexdigest(),
                    "mode": "100644",
                }},
            }},
        }))

    def verify(self):
        return verify_source(self.source, self.pin, self.lock)

    def test_source_archive_without_git_metadata(self):
        self.assertEqual(self.verify(), self.pin)

    def test_modified_source_is_rejected(self):
        self.file.write_text("int value = 2;\n")
        with self.assertRaisesRegex(ValueError, "changed"):
            self.verify()

    def test_missing_source_is_rejected(self):
        self.file.unlink()
        with self.assertRaisesRegex(ValueError, "missing"):
            self.verify()

    def test_added_source_is_rejected(self):
        (self.source / "unreviewed.cpp").write_text("int extra;\n")
        with self.assertRaisesRegex(ValueError, "Unexpected"):
            self.verify()

    def test_changed_executable_mode_is_rejected(self):
        self.file.chmod(0o755)
        with self.assertRaisesRegex(ValueError, "changed"):
            self.verify()

    def test_wrong_revision_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "matching pinned"):
            verify_source(self.source, "b" * 40, self.lock)

    def test_monorepo_git_metadata_does_not_replace_source_manifest(self):
        subprocess.run(["git", "init", "--quiet", str(self.root)], check=True)
        self.lock.unlink()
        with self.assertRaises(FileNotFoundError):
            self.verify()

    def test_generated_cache_does_not_invalidate_source(self):
        cache = self.source / "__pycache__"
        cache.mkdir()
        (cache / "temporary.pyc").write_bytes(b"generated")
        self.assertEqual(self.verify(), self.pin)


if __name__ == "__main__":
    unittest.main()
