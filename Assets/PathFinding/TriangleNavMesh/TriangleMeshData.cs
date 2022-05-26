using System;
using JetBrains.Annotations;
using UnityEngine;

namespace PathFinding.TriangleNavMesh
{
    public class TriangleMeshData
    {
        // public Vector3 size;
        // public Vector3 origin;
        public int[] indices;
        public Vector3[] vertices;

        public TriangleMeshData()
        {
        }

        public TriangleMeshData(int[] indices, Vector3[] vertices)
        {
            // this.size = size;
            // this.origin = origin;
            this.indices = indices;
            this.vertices = vertices;
        }

        public byte[] Serialize()
        {
            using (var stream = new MemoryStream())
            {
                var indicesLength = indices?.Length ?? -1;
                stream.WriteInt32(indicesLength);
                for (int i = 0; i < indicesLength; i++)
                    stream.WriteInt32(indices[i]);

                var verticesLength = vertices?.Length ?? -1;
                stream.WriteInt32(verticesLength);
                for (int i = 0; i < verticesLength; i++)
                for (int j = 0; j < 3; j++)
                    stream.WriteSingle(vertices[i][j]);

                // for (int i = 0; i < 3; i++)
                // stream.WriteSingle(size[i]);
                // for (int i = 0; i < 3; i++)
                // stream.WriteSingle(origin[i]);

                var bytes = stream.ToArray();
                Debug.Log($"Serialize {bytes.Length}");
                return bytes;
            }
        }

        public void Deserialize(byte[] data)
        {
            Debug.Log(data.Length);
            using (var stream = new MemoryStream(data))
            {
                var indicesLength = stream.ReadInt32();
                indices = new int[indicesLength];
                for (int i = 0; i < indicesLength; i++)
                    indices[i] = stream.ReadInt32();
                var verticesLength = stream.ReadInt32();
                vertices = new Vector3[verticesLength];
                for (int i = 0; i < verticesLength; i++)
                for (int j = 0; j < 3; j++)
                    vertices[i][j] = stream.ReadSingle();
                // for (int i = 0; i < 3; i++)
                // size[i] = stream.ReadSingle();
                // for (int i = 0; i < 3; i++)
                // origin[i] = stream.ReadSingle();
            }
        }

        private class MemoryStream : System.IO.MemoryStream
        {
            public MemoryStream()
            {
            }

            public MemoryStream([NotNull] byte[] buffer) : base(buffer)
            {
            }

            public void WriteInt32(int value)
            {
                var bytes = BitConverter.GetBytes(value);
                Write(bytes, 0, bytes.Length);
            }

            public void WriteSingle(float value)
            {
                var bytes = BitConverter.GetBytes(value);
                Write(bytes, 0, bytes.Length);
            }

            public int ReadInt32()
            {
                var bytes = new byte[4];
                Read(bytes, 0, 4);
                return BitConverter.ToInt32(bytes, 0);
            }

            public float ReadSingle()
            {
                var bytes = new byte[4];
                Read(bytes, 0, 4);
                return BitConverter.ToSingle(bytes, 0);
            }
        }
    }
}