"""Allow ``python -m tests`` as an entry point equivalent to ``python test.py``."""
from .cli import main

if __name__ == "__main__":
    main()
