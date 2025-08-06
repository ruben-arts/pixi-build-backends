from typing import List, Optional
from pixi_build_backend.pixi_build_backend import PyConditionalString, PyListOrItemString, PyVecItemPackageDependency, PyItemPackageDependency
from pixi_build_backend.types.conditional import ConditionalPackageDependency
from pixi_build_backend.types.requirements import PackageDependency

class VecItemPackageDependency(List):
    """A wrapper for a list of ItemPackageDependency."""

    _inner: PyVecItemPackageDependency

    def __init__(self, items=None):
        """Initialize with optional items."""
        if items is None:
            self._inner = PyVecItemPackageDependency()
        else:
            # Extract _inner from ItemPackageDependency objects
            inner_items = [item._inner if isinstance(item, ItemPackageDependency) else item for item in items]
            self._inner = PyVecItemPackageDependency(inner_items)

    @classmethod
    def _from_inner(cls, inner: PyVecItemPackageDependency) -> "VecItemPackageDependency":
        """Create from PyVecItemPackageDependency."""
        instance = cls.__new__(cls)
        instance._inner = inner
        return instance

    def __len__(self) -> int:
        """Return the length of the list."""
        return len(self._inner)

    def __getitem__(self, index) -> "ItemPackageDependency":
        """Get item at index."""
        return ItemPackageDependency._from_inner(self._inner[index])

    def __setitem__(self, index, value):
        """Set item at index."""
        inner_value = value._inner if isinstance(value, ItemPackageDependency) else value
        self._inner[index] = inner_value

    def __delitem__(self, index):
        """Delete item at index."""
        del self._inner[index]

    def __iter__(self):
        """Return iterator."""
        for item in self._inner:
            yield ItemPackageDependency._from_inner(item)

    def __contains__(self, item: "ItemPackageDependency") -> bool:
        """Check if item is in the list."""
        inner_item = item._inner if isinstance(item, ItemPackageDependency) else item
        return inner_item in self._inner

    def append(self, item: "ItemPackageDependency"):
        """Append item to the list."""
        inner_item = item._inner
        self._inner.append(inner_item)

    def extend(self, items: List["ItemPackageDependency"]):
        """Extend the list with items."""
        inner_items = [item._inner for item in items]
        self._inner.extend(inner_items)

    def insert(self, index, item: "ItemPackageDependency"):
        """Insert item at index."""
        inner_item = item._inner
        self._inner.insert(index, inner_item)

    def remove(self, item: "ItemPackageDependency"):
        """Remove first occurrence of item."""
        inner_item = item._inner
        self._inner.remove(inner_item)

    def pop(self, index=-1):
        """Remove and return item at index."""
        return ItemPackageDependency._from_inner(self._inner.pop(index))

    def clear(self):
        """Remove all items."""
        self._inner.clear()

    def index(self, item: "ItemPackageDependency", start=0, stop=None):
        """Return index of first occurrence of item."""
        inner_item = item._inner
        if stop is None:
            return self._inner.index(inner_item, start)
        else:
            return self._inner.index(inner_item, start, stop)

    def count(self, item) -> int:
        """Return count of occurrences of item."""
        inner_item = item._inner if isinstance(item, "ItemPackageDependency") else item
        return self._inner.count(inner_item)

    def reverse(self):
        """Reverse the list in place."""
        self._inner.reverse()

    def sort(self, key=None, reverse=False):
        """Sort the list in place."""
        if key is None:
            self._inner.sort(reverse=reverse)
        else:
            self._inner.sort(key=key, reverse=reverse)

    def copy(self):
        """Return a shallow copy."""
        return VecItemPackageDependency._from_inner(self._inner.copy())

    def __eq__(self, other) -> bool:
        """Check equality."""
        if isinstance(other, VecItemPackageDependency):
            return self._inner == other._inner
        return False

    def __str__(self) -> str:
        """Return string representation."""
        return str(self._inner)
    

class ItemPackageDependency:
    """A package dependency item wrapper."""

    _inner: PyItemPackageDependency

    def __init__(self, name: str):
        self._inner = PyItemPackageDependency(name)

    @classmethod
    def new_from_conditional(
        cls,
        conditional: ConditionalPackageDependency
    ) -> "ItemPackageDependency":
        new_class = cls.__new__(cls)
        new_class._inner = PyItemPackageDependency.new_from_conditional(conditional._inner)
        return new_class


    @classmethod
    def _from_inner(cls, inner: PyItemPackageDependency) -> "ItemPackageDependency":
        """Create an ItemPackageDependency from a FFI PyItemPackageDependency."""
        instance = cls.__new__(cls)
        instance._inner = inner
        return instance

    def __str__(self):
        return str(self._inner)
    
    @property
    def concrete(self) -> "PackageDependency":
        """Get the concrete package dependency."""
        return PackageDependency._from_inner(self._inner.concrete())
    
    @property
    def template(self) -> Optional[str]:
        """Get the template string if this is a template."""
        return self._inner.template()
    
    @property
    def conditional(self) -> Optional[ConditionalPackageDependency]:
        """Get the conditional string if this is a conditional."""
        return self._inner.conditional()