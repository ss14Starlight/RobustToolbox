using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Server.GameObjects;
using Robust.Shared;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Player;

namespace Robust.UnitTesting.Server.GameStates._Starlight;

public sealed class RemoteViewPvsFilterTest : RobustIntegrationTest
{
    [Test]
    public async Task PvsFilterIncludesRemoteViewSubscription()
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;

        var entityManager = server.ResolveDependency<IEntityManager>();
        var playerManager = server.ResolveDependency<ISharedPlayerManager>();

        await RunTicksSync(server, client, 10);

        await server.WaitPost(() =>
        {
            var maps = entityManager.System<SharedMapSystem>();
            var localMap = maps.CreateMap(out var localMapId);
            var remoteMap = maps.CreateMap(out var remoteMapId);
            var localEntity = entityManager.SpawnEntity(null, new EntityCoordinates(localMap, Vector2.Zero));
            var remoteView = entityManager.SpawnEntity(null, new EntityCoordinates(remoteMap, Vector2.Zero));
            var session = playerManager.Sessions.First();
            var remotePosition = new MapCoordinates(Vector2.Zero, remoteMapId);

            server.PlayerMan.SetAttachedEntity(session, localEntity);
            playerManager.JoinGame(session);

            var filterBeforeSubscription = Filter.Empty().AddInRange(
                remotePosition, 1f, playerManager, entityManager);
            Assert.That(filterBeforeSubscription.Recipients, Does.Not.Contain(session));

            server.System<ViewSubscriberSystem>().AddViewSubscriber(remoteView, session);

            var filterWithSubscription = Filter.Empty().AddInRange(
                remotePosition, 1f, playerManager, entityManager);
            Assert.That(filterWithSubscription.Recipients, Does.Contain(session));

            var localFilter = Filter.Empty().AddInRange(
                new MapCoordinates(Vector2.Zero, localMapId), 1f, playerManager, entityManager);
            Assert.That(localFilter.Recipients, Does.Contain(session));
        });
    }
}
