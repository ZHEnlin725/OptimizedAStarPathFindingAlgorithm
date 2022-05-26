using System;
using System.Collections.Generic;

namespace PathFinding.Optimized
{
    public class FastBinaryHeap<T> : BinaryHeap<T>
    {
        private readonly Dictionary<T, int> _indexes;

        public FastBinaryHeap(HeapType heapType, int capacity = 1) : base(heapType, capacity)
        {
            _indexes = new Dictionary<T, int>(capacity);
        }

        public FastBinaryHeap(IComparer<T> comparer, HeapType heapType, int capacity = 1) : base(comparer, heapType,
            capacity)
        {
            _indexes = new Dictionary<T, int>(capacity);
        }

        public FastBinaryHeap(Func<T, T, int> compareFunc, HeapType heapType, int capacity = 1) : base(compareFunc,
            heapType, capacity)
        {
            _indexes = new Dictionary<T, int>(capacity);
        }

        protected override void Insert(int index, T value)
        {
            base.Insert(index, value);
            if (!IsValueType) _indexes[value] = index;
        }

        public override void Remove(T v)
        {
            if (!IsValueType)
            {
                if (_indexes.TryGetValue(v, out var index))
                    RemoveAt(index);
            }
            else
            {
                base.Remove(v);
            }
        }

        public override T RemoveAt(int index)
        {
            var result = base.RemoveAt(index);
            if (result != null && !IsValueType)
                _indexes.Remove(result);
            return result;
        }

        public override void Clear(bool total = false)
        {
            base.Clear(total);
            _indexes.Clear();
        }
    }
}