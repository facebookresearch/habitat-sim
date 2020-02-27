

class LinkedListNode:
    def __init__(self, key=None, prev_node=None, next_node=None, data=None):
        self.key = key
        self.prev_node = prev_node
        self.next_node = next_node
        self.data = data


class ExtractorLRUCache:
    def __init__(self, capacity=1000):
        self.node_map = {}
        self.head = LinkedListNode()
        self.tail = LinkedListNode(self.head)
        self.head.next_node = self.tail
        self.capacity = capacity
        self.size = 0
    
    def __getitem__(self, key):
        if self.__contains__(key):
            node = self.node_map[key]

            # Accessing the data should move it to front of cache
            self.remove(key)
            self.add(key, node.data)
            return node.data
        else:
            raise KeyError('Key {} not in extractor cache'.format(key))

    def __contains__(self, key):
        return key in self.node_map

    def __str__(self):
        return self.node_map.__str__()

    def remove(self, key):
        if self.__contains__(key):
            cur_node = self.node_map[key]
            prev_node, next_node = cur_node.prev_node, cur_node.next_node
            prev_node.next_node = next_node
            next_node.prev_node = prev_node
            cur_node.next_node = None
            cur_node.prev_node = None
            self.node_map[key] = None
            del self.node_map[key]
            self.size = max(0, self.size - 1)

    def add(self, key, sample):
        if key in self.node_map:
            return
        
        if self.size >= self.capacity:
            self.remove_from_back()

        new_node = LinkedListNode(key, self.head, self.head.next_node, sample)
        self.head.next_node.prev_node = new_node
        self.head.next_node = new_node
        self.node_map[key] = new_node
        self.size += 1
        
    def remove_from_back(self):
        if self.size == 0:
            return
        
        key_to_remove = self.tail.prev_node.key
        self.remove(key_to_remove)

    def print_cache(self):
        out = []
        cur_node = self.head.next_node
        for _ in range(self.size):
            out.append(cur_node.key)
            cur_node = cur_node.next_node

        print(out)
