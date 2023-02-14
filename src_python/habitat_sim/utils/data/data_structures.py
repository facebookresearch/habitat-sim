# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import collections


class ExtractorLRUCache:
    def __init__(self, capacity=1000):
        self._order = collections.OrderedDict()
        self.capacity = capacity
        self.size = 0

    def __getitem__(self, key):
        if not self.__contains__(key):
            raise KeyError("Key {} not in extractor cache".format(key))

        # Accessing the data should move it to front of cache
        k, data = self._order.pop(key)
        self._order[key] = (k, data)
        return data

    def __contains__(self, key):
        return key in self._order

    def __str__(self):
        return self._order.__str__()

    def remove(self, key):
        if self.__contains__(key):
            self._order.pop(key)
            self.size = max(0, self.size - 1)

    def add(self, key, sample):
        if key in self._order:
            self._order.pop(key)

        if self.size >= self.capacity:
            self.remove_from_back()

        value = (key, sample)
        self._order[key] = value
        self.size += 1

    def remove_from_back(self):
        if self.size == 0:
            return

        self._order.popitem(last=False)
        self.size = max(0, self.size - 1)
