package frcsim_physics;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class VendorJNITest {

    @Test
    void jniLinkTest() {
        VendorJNI.initialize();

        long world = VendorJNI.createWorld(0.01, true);
        assertTrue(world != 0, "World handle should be non-zero after successful creation");

        int body = VendorJNI.createBody(world, 1.0);
        assertTrue(body >= 0, "Body index should be non-negative after successful creation");

        VendorJNI.setBodyPosition(world, body, 0.0, 0.0, 1.0);
        VendorJNI.stepWorld(world, 10);

        double[] pos = new double[3];
        int rc = VendorJNI.getBodyPosition(world, body, pos);
        assertTrue(rc == 0, "getBodyPosition should return 0 on success");
        assertTrue(pos[2] < 1.0, "Body should have fallen below initial height due to gravity after 10 steps");

        VendorJNI.destroyWorld(world);
    }
}
