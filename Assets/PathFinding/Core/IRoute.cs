namespace PathFinding.Core
{
    public interface IRoute<T>
    {
        float Cost { get; }

        T Origin { get; }

        T Dest { get; }
    }
}