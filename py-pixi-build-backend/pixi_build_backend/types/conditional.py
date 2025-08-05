from typing import List, Optional, Union
from pixi_build_backend.pixi_build_backend import PyConditionalString, PyListOrItemString




# need to type stub it
ListOrItemString = PyListOrItemString

# class ListOrItemString:
#     """A wrapper for a ListOrItem String container string."""
#     _inner: PyListOrItemString

#     def __init__(self, items: Union[List[str], str]) -> None:

#         items = items if isinstance(items, list) else [items]

#         self._inner = PyListOrItemString(items)

    
#     def __str__(self) -> str:
#         """Get the string representation."""
#         return str(self._inner)
    
#     def __eq__(self, other: object) -> bool:
#         """Check equality."""
#         if not isinstance(other, ListOrItemString):
#             return False
#         return self._inner == other._inner
    
#     def __getitem__(self, index: int) -> str:
#         """Get item by index."""
#         return self._inner[index]
    
#     def __len__(self) -> int:
#         """Get the number of items in the container."""
#         return len(self._inner)
    
#     def __iter__(self):
#         """Iterate over items in the container."""
#         for i in range(len(self)):
#             yield self._inner[i]
    
#     def __contains__(self, item: str) -> bool:
#         """Check if item is in the container."""
#         return item in self._inner
    
#     def append(self, item: str) -> None:
#         """Add an item to the end of the container."""
#         self._inner.append(item)
    
#     def extend(self, items: List[str]) -> None:
#         """Add multiple items to the end of the container."""
#         self._inner.extend(items)
    
#     def insert(self, index: int, item: str) -> None:
#         """Insert an item at the specified index."""
#         self._inner.insert(index, item)
    
#     def remove(self, item: str) -> None:
#         """Remove the first occurrence of item."""
#         self._inner.remove(item)
    
#     def pop(self, index: int = -1) -> str:
#         """Remove and return item at index (default last)."""
#         return self._inner.pop(index)
    
#     def index(self, item: str, start: int = 0, end: Optional[int] = None) -> int:
#         """Return the index of the first occurrence of item."""
#         return self._inner.index(item, start, end)
    
#     def count(self, item: str) -> int:
#         """Return the number of times item appears in the container."""
#         return self._inner.count(item)
    
#     def clear(self) -> None:
#         """Remove all items from the container."""
#         self._inner.clear()
    
#     def reverse(self) -> None:
#         """Reverse the items in place."""
#         self._inner.reverse()

# ListOrItemString = List[str]

class ConditionalString:
    _inner: PyConditionalString

    def __init__(self, condition: str, then: ListOrItemString, else_: Optional[ListOrItemString]) -> None:
        else_ = else_ if else_ is not None else ListOrItemString([])
        self._inner = PyConditionalString(condition, then, else_)

    @property
    def condition(self) -> str:
        """Get the condition string."""
        return self._inner.condition

    @property
    def then_value(self) -> List[str]:
        """Get the then value."""
        # result = ListOrItemString([])
        return self._inner.then_value
        # return result

    @property
    def else_value(self) -> ListOrItemString:
        """Get the else value."""
        return self._inner.else_value
        # result = ListOrItemString([])
        # result._inner = self._inner.else_value
        # return result
    
    def __str__(self):
        return str(self._inner)
    
    def __eq__(self, other: object) -> bool:
        """Check equality."""
        if not isinstance(other, ConditionalString):
            return False
        return self._inner == other._inner



class ConditionalPackageDepedency:
    pass