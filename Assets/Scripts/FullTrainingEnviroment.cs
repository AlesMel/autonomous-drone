using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Pool;

public class FullTrainingEnviroment : MonoBehaviour
{
    private IObjectPool<FullTrainingEnviroment> objectPool;

    // public property to give the projectile a reference to its ObjectPool
    public IObjectPool<FullTrainingEnviroment> ObjectPool { set => objectPool = value; }

    public void Deactivate()
    {
        StartCoroutine(DeactivateRoutine(0.0f));
    }

    IEnumerator DeactivateRoutine(float delay)
    {
        yield return new WaitForSeconds(delay);

        // release the projectile back to the pool
        objectPool.Release(this);
    }
}
