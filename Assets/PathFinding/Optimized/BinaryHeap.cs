using System;
using System.Collections.Generic;

namespace PathFinding.Optimized
{
    public class BinaryHeap<T>
    {
        public delegate bool WalkFunc(T value);

        public enum HeapType : byte
        {
            Minimum,
            Maximum
        }

        private const int Top = 1;

        private int _tail;
        private T[] _array;
        private readonly HeapType _heapType;
        private readonly Func<T, T, int> _compareFunc;

        public int Count
        {
            get
            {
                lock (_array)
                {
                    return _tail;
                }
            }
        }

        protected bool IsValueType { get; }

        public BinaryHeap(HeapType heapType, int capacity = 1) : this((x, y) => ((IComparable<T>) x).CompareTo(y),
            heapType, capacity)
        {
            if (!typeof(IComparable<T>).IsAssignableFrom(typeof(T)))
                throw new ArgumentException($"{typeof(IComparable)} Must Assignable From {typeof(T)}");
        }

        public BinaryHeap(IComparer<T> comparer, HeapType heapType, int capacity = 1) : this(comparer.Compare,
            heapType, capacity)
        {
        }

        public BinaryHeap(Func<T, T, int> compareFunc, HeapType heapType, int capacity = 1)
        {
            _compareFunc = compareFunc;
            _heapType = heapType;
            _array = new T[capacity];
            IsValueType = typeof(T).IsValueType;
        }

        public virtual T Peek()
        {
            lock (_array)
            {
                return _tail < Top ? default : _array[Top];
            }
        }

        public virtual T Pop() => RemoveAt(Top);

        public virtual void Push(T v)
        {
            lock (_array)
            {
                EnsureCapacity(++_tail + Top);
                Insert(_tail, v);
                Upwards(_tail);
            }
        }

        public virtual void Remove(T v)
        {
            for (var i = Top; i < _tail; i++)
            {
                if (ReferenceEquals(v, _array[i]) || _compareFunc(v, _array[i]) == 0)
                {
                    RemoveAt(i);
                    break;
                }
            }
        }

        public virtual T RemoveAt(int index)
        {
            lock (_array)
            {
                if (index < Top || index > _tail)
                    return default;
                var result = _array[index];
                Swap(index, _tail--);
                Downwards(index);
                return result;
            }
        }

        public virtual void Clear(bool total = false)
        {
            lock (_array)
            {
                _tail = 0;
                if (total) _array = new T[1];
            }
        }

        private readonly Stack<int> _walkStack = new Stack<int>();

        public virtual void Walk(int parent, WalkFunc func) //前序遍历
        {
            if (parent > _tail || parent < Top) return;
            _walkStack.Clear();
            _walkStack.Push(parent);
            while (_walkStack.Count > 0)
            {
                var index = _walkStack.Pop();
                if (index > _tail) break;
                if (func(_array[index])) break;
                var left = index << 1;
                if (left > _tail) continue;
                var right = left + 1;
                if (right > _tail)
                {
                    _walkStack.Push(left);
                    continue;
                }

                _walkStack.Push(right);
                _walkStack.Push(left);
            }
        }

        private void Upwards(int index)
        {
            while (index > Top)
            {
                var parent = index >> 1;
                if (!Swappable(index, parent)) break;
                Swap(index, parent);
                index = parent;
            }
        }

        private void Downwards(int index)
        {
            while (index < _tail)
            {
                int left = index << 1, right = left + 1;
                if (left > _tail) break;
                var swap = right <= _tail ? (Swappable(right, left) ? right : left) : left;
                if (!Swappable(swap, index)) break;
                Swap(swap, index);
                index = swap;
            }
        }

        private void EnsureCapacity(int size)
        {
            var length = _array.Length;
            if (size > length)
            {
                while (size > length)
                    length = (length << 1) + 1;
                var dest = new T[length];
                Array.Copy(_array, dest, _array.Length);
                _array = dest;
            }
        }

        private void Swap(int index1, int index2)
        {
            var temp = _array[index2];
            Insert(index2, _array[index1]);
            Insert(index1, temp);
        }

        private bool Swappable(int candidate, int refer)
        {
            var result = _compareFunc(_array[candidate], _array[refer]);
            return _heapType == HeapType.Maximum ? result > 0 : result < 0;
        }

        protected virtual void Insert(int index, T value) => _array[index] = value;
    }
}