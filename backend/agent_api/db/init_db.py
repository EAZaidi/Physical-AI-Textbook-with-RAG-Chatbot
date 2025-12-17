"""Database initialization script.

Creates the conversation_threads table with the required schema.
"""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from agent_api.sessions import Base, engine
from sqlalchemy import inspect


def init_db():
    """Initialize database schema."""
    print("Creating database schema...")
    
    # Create all tables
    Base.metadata.create_all(bind=engine)
    
    # Verify tables were created
    inspector = inspect(engine)
    tables = inspector.get_table_names()
    
    if "conversation_threads" in tables:
        print("✓ Table 'conversation_threads' created successfully")
        
        # Print column info
        columns = inspector.get_columns("conversation_threads")
        print("\nTable schema:")
        for col in columns:
            print(f"  - {col['name']}: {col['type']}")
        
        # Print indexes
        indexes = inspector.get_indexes("conversation_threads")
        if indexes:
            print("\nIndexes:")
            for idx in indexes:
                print(f"  - {idx['name']}: {idx['column_names']}")
    else:
        print("✗ Failed to create 'conversation_threads' table")
        return False
    
    print("\nDatabase initialization complete.")
    return True


if __name__ == "__main__":
    try:
        success = init_db()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"Error initializing database: {e}")
        sys.exit(1)
