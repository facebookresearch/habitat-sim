import collections
from typing import Dict, Tuple, Union

from numpy import float64, ndarray


class ExtractorLRUCache:
    def __init__(self, capacity: int = 1000) -> None:
        self._order: collections.OrderedDict[
            int, Tuple[int, str]
        ] = collections.OrderedDict()
        self.capacity = capacity
        self.size = 0

    def __getitem__(self, key: int) -> str:
        if self.__contains__(key):
            # Accessing the data should move it to front of cache
            k, data = self._order.pop(key)
            self._order[key] = (k, data)
            return data
        else:
            raise KeyError("Key {} not in extractor cache".format(key))

    def __contains__(self, key: Union[int, Tuple[int, str]]) -> bool:
        return key in self._order

    def __str__(self):
        return self._order.__str__()

    def remove(self, key: int):
        if self.__contains__(key):
            self._order.pop(key)
            self.size = max(0, self.size - 1)

    def add(
        self,
        key: Union[int, Tuple[int, str]],
        sample: Union[str, Dict[str, Union[ndarray, float64]]],
    ) -> None:
        if key in self._order:
            self._order.pop(key)  # type: ignore

        if self.size >= self.capacity:
            self.remove_from_back()

        value = (key, sample)
        self._order[key] = value  # type: ignore
        self.size += 1

    def remove_from_back(self) -> None:
        if self.size == 0:
            return

        self._order.popitem(last=False)
        self.size = max(0, self.size - 1)
