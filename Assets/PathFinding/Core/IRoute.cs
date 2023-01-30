namespace PathFinding.Core
{
    public interface IRoute<T>
    {
        float cost { get; }

        T origin { get; }

        T dest { get; }
    }
}