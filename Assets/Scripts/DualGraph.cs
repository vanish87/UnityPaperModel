using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityTools.Algorithm;
using UnityTools.Common;
using UnityTools.Debuging;
using UnityTools.Debuging.EditorTool;

namespace UnityPaperModel
{
    public class VertexGraph : NewGraph<VertexGraph.Vertex, VertexGraph.Edge, VertexGraph.GraphFactory>
    {
        public class GraphFactory : IGraphFactory
        {
            public IEdge CreateEdge(IVertex v1, IVertex v2, bool isDirectional = false)
            {
                return new Edge() { Vertex = v1, OtherVertex = v2 };
            }

            public INewGraph CreateGraph()
            {
                return new VertexGraph();
            }

            public IVertex CreateVertex()
            {
                return new Vertex();
            }
        }
        public class CloseVertex : IVertex
        {
            public float3 Position { get => this.position; set => this.position = value; }
            protected float3 position;
            public object Clone()
            {
                return new CloseVertex() { position = this.position };
            }

            public bool Equals(IVertex other)
            {
                var obj = other as CloseVertex;
                if(obj == null) return false;

                return math.distancesq(this.position, obj.position) < 0.001f;
            }

            public override bool Equals(object other)
            {
                if(other is IVertex) return this.Equals(other as IVertex);
                return base.Equals(other);
            }

            public override int GetHashCode()
            {
                return string.Format("{0:F2}{1:F2}{2:F2}", this.Position.x, this.Position.y, this.Position.z).GetHashCode();
            }
        }
        public class Vertex : IVertex, IPoint
        {
            public float3 Position { get => this.position; set => this.position = value; }
            protected float3 position;
            public virtual object Clone()
            {
                return new Vertex() { position = this.Position };
            }

            public virtual bool Equals(IVertex other)
            {
                return math.distance(this.Position, (other as Vertex).Position) < 0.001f;
                //return base.Equals(other);
            }
            
        }

        public class Edge : DefaultEdge
        {
            public float Length
            {
                get
                {
                    var v1 = (this.Vertex as Vertex).Position;
                    var v2 = (this.OtherVertex as Vertex).Position;

                    return math.distance(v1, v2);
                }
            }
           public override object Clone()
            {
                return new Edge(){ isDirectional = this.IsDirectional, Vertex = this.Vertex.Clone() as IVertex, OtherVertex = this.OtherVertex.Clone() as IVertex };
            }
            public override string ToString()
            {
                return (this.Vertex as Vertex).Position + " " + (this.OtherVertex as Vertex).Position;
            }
        }

        public void OnDrawGizmos()
        {
            foreach(var v in this.Vertices)
            {
                Gizmos.DrawSphere(v.Position, 0.01f);
            }

            foreach (var e in this.Edges)
            {
                var p1 = e.Vertex as Vertex;
                var p2 = e.OtherVertex as Vertex;
                Gizmos.DrawLine(p1.Position, p2.Position);
            }
        }

    }

    public class DualGraph : NewGraph<DualGraph.Face, DualGraph.Edge, DualGraph.GraphFactory>
    {
        public class GraphFactory : IGraphFactory
        {
            protected int currentID = 0;
            public IEdge CreateEdge(IVertex v1, IVertex v2, bool isDirectional = false)
            {
                return new Edge() { Vertex = v1, OtherVertex = v2 };
            }

            public INewGraph CreateGraph()
            {
                return new DualGraph();
            }

            public IVertex CreateVertex()
            {
                return new Face() { index = this.currentID++ };
            }
        }
        public class Face : IndexVertex
        {
            public float3 Center => this.center;
            public float3 Normal { get => this.normal; internal set => this.normal = value; }

            public Matrix4x4 LocalMat { get => this.localRotationMat; set => this.localRotationMat = value; }
            protected HashSet<IEdge> edges = new HashSet<IEdge>();
            protected float3 sum = float3.zero;
            protected float3 center = float3.zero;
            protected float3 normal = float3.zero;
            protected Matrix4x4 localRotationMat = Matrix4x4.identity;
            
            public void AddEdge(VertexGraph.Edge e)
            {
                if(this.edges.Add(e))
                {
                    this.sum += (e.Vertex as VertexGraph.Vertex).Position;
                    this.sum += (e.OtherVertex as VertexGraph.Vertex).Position;
                }
                this.center = this.sum / (this.edges.Count * 2);
            }

            public IEdge GetSharedEdgeWith(Face face)
            {
                foreach(var e in this.edges)
                {
                    if(face.ContainsGeometryEdge(e)) return e;
                }
                return default;                
            }

            public float2 GetMinMaxEdgeLength()
            {
                var ret = new float2(float.MaxValue, 0);
                foreach(var e in this.edges)
                {
                    var len =(e as VertexGraph.Edge).Length;
                    ret.x = len < ret.x ? len : ret.x;
                    ret.y = len > ret.y ? len : ret.y;
                }

                return ret;
            }
            public void Transform(Matrix4x4 mat)
            {
                this.sum = 0;
                this.center = 0;
                
                foreach(var v in this.edges)
                {
                    var from = v.Vertex as VertexGraph.Vertex;
                    var to = v.OtherVertex as VertexGraph.Vertex;

                    from.Position = mat.MultiplyPoint(from.Position);
                    to.Position = mat.MultiplyPoint(to.Position);                   

                    this.sum += from.Position + to.Position;
                }
                this.center = this.sum / (this.edges.Count * 2);
                this.normal = mat.MultiplyVector(this.normal);
            }
            public void FlatToYZero()
            {
                foreach(var e in this.edges)
                {
                    var from = e.Vertex as VertexGraph.Vertex;
                    var to = e.OtherVertex as VertexGraph.Vertex;

                    from.Position = new float3(from.Position.x, 0, from.Position.z);
                    to.Position = new float3(to.Position.x, 0, to.Position.z);
                }
                this.center.y = 0;
            }
            public bool ContainsGeometryEdge(IEdge v)
            {
                return this.edges.Contains(v);
            }

            public bool IsIntersectWith(Face other)
            {
                foreach (var e in this.edges)
                {
                    foreach(var o in other.edges)
                    {
                        if(e.Equals(o) || e == o) continue;
                        if(e.Vertex.Equals(o.Vertex) || e.OtherVertex.Equals(o.OtherVertex)) continue;
                        if(e.Vertex.Equals(o.OtherVertex) || e.OtherVertex.Equals(o.Vertex)) continue;

                        var a1 = e.Vertex as IPoint;
                        var a2 = e.OtherVertex as IPoint;
                        var b1 = o.Vertex as IPoint;
                        var b2 = o.OtherVertex as IPoint;

                        var p1 = new float2(a1.Position.xz);
                        var p2 = new float2(a2.Position.xz);
                        var p3 = new float2(b1.Position.xz);
                        var p4 = new float2(b2.Position.xz);

                        if (GeometryTools.AreLinesIntersecting(p1, p2, p3, p4, false)) return true;

                        // if (GeometryTools.AreLineSegmentsIntersectingDotProduct(a1.Position, a2.Position, b1.Position, b2.Position)) return true;
                    }
                }

                return false;
            }
            public override object Clone()
            {
                var ret = new Face()
                {
                    index = this.index,
                    sum = this.sum,
                    center = this.center,
                    localRotationMat = this.localRotationMat,
                    normal = this.normal,
                    edges = new HashSet<IEdge>(this.edges.Select(e => e.Clone() as IEdge))
                };
                

                return ret;
            }
            public override int GetHashCode()
            {
                return base.GetHashCode();
                // return string.Format("{0:F2}{1:F2}{2:F2}", this.Center.x, this.Center.y, this.Center.z).GetHashCode();
            }
            
            public override bool Equals(object other)
            {
                if(other is IVertex) return this.Equals(other as IVertex);
                return base.Equals(other);
            }

            public override bool Equals(IVertex other)
            {
                return base.Equals(other);
            }
            public void OnDrawGizmos()
            {
                var c= Expand(this.Center);
                Gizmos.DrawSphere(c, 0.01f);
                Gizmos.DrawLine(c, c + this.Normal);

                foreach(var e in this.edges)
                {
                    var v1 = (e.Vertex as VertexGraph.Vertex).Position;
                    var v2 =  (e.OtherVertex as VertexGraph.Vertex).Position;
                    Gizmos.DrawLine(Expand(v1), Expand(v2));
                }

            }

            protected float3 Expand(float3 v)
            {
                return v + normal * 0;
            }
            
        }

        public class Edge : DefaultEdge, IWeightedEdge, MinimumSetCover.ICost
        {
            public float Weight { get => this.weight; set => this.weight = value; }

            public float Cost => (1 - this.gamma) * (1 - this.Weight) + this.gamma;

            protected float weight = 0;
            protected float gamma = 0.5f;

            public override object Clone()
            {
                return new Edge(){ isDirectional = this.IsDirectional, Vertex = this.Vertex.Clone() as IVertex, OtherVertex = this.OtherVertex.Clone() as IVertex };
            }
            
        }
    
        public void UpdateWeight(float3 diriedDirection, float beta = 0.5f)
        {
            var globalMinMax = GetMinMaxGeoEdgeLength(this);
            foreach(var e in this.Edges)
            {
                var from = e.Vertex as Face;
                var to = e.Vertex as Face;
                var sharedEdge = from.GetSharedEdgeWith(to) as VertexGraph.Edge;
                var sharedDir = (sharedEdge.Vertex as VertexGraph.Vertex).Position - (sharedEdge.OtherVertex as VertexGraph.Vertex).Position;
                e.Weight = (1 - beta) * this.GetM(sharedEdge.Length, globalMinMax) + beta * this.GetF(diriedDirection, sharedDir);
            }
        }

        public List<(Face, Face)> GetOverlapFace()
        {
            var ret = new List<(Face, Face)>();


            foreach(var v in this.Vertices)
            {
                foreach(var other in this.Vertices)
                {
                    if(v == other) continue;

                    if(v.IsIntersectWith(other)) ret.Add((v, other));
                }
            }

            return ret;
        }
        

        protected float GetM(float length, float2 minMax)
        {
            return 1 - (length - minMax.x) / (minMax.y - minMax.x);
        }
        protected float GetF(float3 dir, float3 edgeDir)
        {
            return math.abs(math.dot(dir, edgeDir)) / math.length(edgeDir);
        }

        public static Matrix4x4 GetLocalRotationMatrix(Face from, Face to, bool normalOnly = false)
        {
            var org = float3.zero;
            var trans = Matrix4x4.identity;
            var transback= Matrix4x4.identity;

            if(normalOnly == false)
            {
                var sharedEdge = from.GetSharedEdgeWith(to);
                LogTool.AssertNotNull(sharedEdge);

                org = (sharedEdge.Vertex as VertexGraph.Vertex).Position;
                trans = Matrix4x4.Translate(org);
                transback = Matrix4x4.Translate(-org);
            }
            var n1 = from.Normal;
            var n2 = to.Normal;
            var axis = math.cross(n2, n1);

            var cos = math.dot(n1, n2);
            var angle = math.acos(cos) * Mathf.Rad2Deg;

            return trans * Matrix4x4.Rotate(Quaternion.AngleAxis(angle, axis)) * transback;
        }

        public static void CalculateLocalMatrix(DualGraph graph, Face node, HashSet<IVertex> visited)
        {
            if(visited.Contains(node)) return;
            visited.Add(node);
            
            foreach(Face next in graph.GetNeighborVertices(node))            
            {
                if(visited.Contains(next)) continue;

                next.LocalMat = GetLocalRotationMatrix(node, next);
                CalculateLocalMatrix(graph, next, visited);
            }

        }

        public static void FlatGraph(DualGraph graph, Face node, HashSet<IVertex> visited, Matrix4x4 parent)
        {
            if(visited.Contains(node)) return;
            visited.Add(node);

            node.Transform(parent * node.LocalMat);

            foreach(Face next in graph.GetNeighborVertices(node))            
            {
                FlatGraph(graph, next, visited, parent * node.LocalMat);
            }
        }

        public void MoveToYZero()
        {
            foreach(Face f in this)
            {
                f.FlatToYZero();                
            }
        }

        public Face FindFaceSharesEdgeWith(Face face, VertexGraph.Edge edge)
        {
            foreach(var f in this.Vertices)
            {
                if(f != face && f.ContainsGeometryEdge(edge))
                {
                    return f;
                }
            }
            return default;
        }

        public static float2 GetMinMaxGeoEdgeLength(DualGraph graph)
        {
            var ret = new float2(float.MaxValue, 0);

            foreach(var face in graph.Vertices)
            {
                var len = face.GetMinMaxEdgeLength();
                ret.x = len.x < ret.x ? len.x : ret.x;
                ret.y = len.y > ret.y ? len.y : ret.y;
            }
            return ret;
        }

        public void OnDrawGizmos()
        {
            using (new GizmosScope(Color.cyan, Matrix4x4.identity))
            {
                foreach (var v in this.Vertices)
                {
                    v.OnDrawGizmos();
                }
            }
            using (new GizmosScope(Color.blue, Matrix4x4.identity))
            {
                foreach (var e in this.Edges)
                {
                    var p1 = e.Vertex as Face;
                    var p2 = e.OtherVertex as Face;
                    Gizmos.DrawLine(p1.Center, p2.Center);
                }
            }
        }
    }
    
}