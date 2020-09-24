using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
namespace UnityPaperModel
{
    public class DualGraphMono : MonoBehaviour
    {
        [SerializeField] protected Mesh mesh;
        [SerializeField] protected int2 index;
        protected DualGraph dualGraph;
        protected void Start()
        {
            this.mesh = this.GetComponent<MeshFilter>().mesh;
            // this.dualGraph = new DualGraph(this.mesh);

            this.dualGraph = DualGraph.Load();
        }
        protected void Update()
        {
            if(Input.GetKeyDown(KeyCode.R))
            {
                var edge = this.dualGraph.GetDualGraph.GetEdge(index.x, index.y);
                if(edge != null) this.dualGraph.GetDualGraph.RemoveEdge(edge);
            }
            if(Input.GetKeyDown(KeyCode.S)) DualGraph.Save(this.dualGraph);
            if(Input.GetKeyDown(KeyCode.L)) this.dualGraph = DualGraph.Load();
        }

        protected void OnDrawGizmos()
        {
            this.dualGraph?.OnDrawGizmos();
        }
    }
}