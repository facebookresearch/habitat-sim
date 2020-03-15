import collections


class ExtractorLRUCache:
    def __init__(self, capacity=1000):
        self._order = collections.OrderedDict()
        self.capacity = capacity
        self.size = 0

    def __getitem__(self, key):
        if self.__contains__(key):
            k, data = self._order[key]

            # Accessing the data should move it to front of cache
            self.remove(key)
            self.add(key, data)
            return data
        else:
            raise KeyError("Key {} not in extractor cache".format(key))

    def __contains__(self, key):
        return key in self._order

    def __str__(self):
        return self._order.__str__()

    def remove(self, key):
        if self.__contains__(key):
            del self._order[key]
            self.size = max(0, self.size - 1)

    def add(self, key, sample):
        if key in self._order:
            return

        if self.size >= self.capacity:
            self.remove_from_back()

        value = (key, sample)
        self._order[key] = value
        self.size += 1

    def remove_from_back(self):
        if self.size == 0:
            return

        self._order.popitem()

    def print_cache(self):
        print(self._order)
