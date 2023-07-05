public interface IOpenList<TElement>
{
    bool isEmpty { get; }

    void Push(TElement val);

    TElement Pop();

    void Remove(TElement val);

    void Clear(bool release = false);
}