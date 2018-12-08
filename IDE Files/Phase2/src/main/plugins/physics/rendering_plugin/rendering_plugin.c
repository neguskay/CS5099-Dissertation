#include <ode/ode.h>
#include <plugins/physics.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

int received_goal = 0;
double goal[2];

int num_samples = 0;
int allocated_samples = 0;

typedef struct {
  double x;
  double y;
  double red;
  double green;
  double blue;
} coord;

coord* samples;

/*
 * Note: This plugin will become operational only after it was compiled and associated with the current world (.wbt).
 * To associate this plugin with the world follow these steps:
 *  1. In the Scene Tree, expand the "WorldInfo" node and select its "physics" field
 *  2. Then hit the [Select] button at the bottom of the Scene Tree
 *  3. In the list choose the name of this plugin (same as this file without the extention)
 *  4. Then save the .wbt by hitting the "Save" button in the toolbar of the 3D view
 *  5. Then revert the simulation: the plugin should now load and execute with the current simulation
 */

void webots_physics_init() {
  /*
   * Get ODE object from the .wbt model, e.g.
   *   dBodyID body1 = dWebotsGetBodyFromDEF("MY_ROBOT");
   *   dBodyID body2 = dWebotsGetBodyFromDEF("MY_SERVO");
   *   dGeomID geom2 = dWebotsGetGeomFromDEF("MY_SERVO");
   * If an object is not found in the .wbt world, the function returns NULL.
   * Your code should correcly handle the NULL cases because otherwise a segmentation fault will crash Webots.
   *
   * This function is also often used to add joints to the simulation, e.g.
   *   dWorldID world = dBodyGetWorld(body1);
   *   dJointID joint = dJointCreateBall(world, 0);
   *   dJointAttach(joint, body1, body2);
   *   ...
   */
}

void webots_physics_step() {
  /*
   * Do here what needs to be done at every time step, e.g. add forces to bodies
   *   dBodyAddForce(body1, f[0], f[1], f[2]);
   *   ...
   */
}

// From http://stackoverflow.com/a/9210560
char** str_split(char* a_str, const char a_delim)
{
    char** result    = 0;
    size_t count     = 0;
    char* tmp        = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;

    result = malloc(sizeof(char*) * count);

    if (result)
    {
        size_t idx  = 0;
        char* token = strtok(a_str, delim);

        while (token)
        {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        assert(idx == count - 1);
        *(result + idx) = 0;
    }

    return result;
}

void process_message(char* message) {
  char** tokens = str_split(message, ',');
  if (tokens) {
    // Goal update message.
    if (strcmp(tokens[0], "G") == 0) {
      double x = strtod(tokens[1], NULL);
      double y = strtod(tokens[2], NULL);
      goal[0] = x;
      goal[1] = y;
      received_goal = 1;
    }
    // Sample point update or current path point messages.
    /*else if (strcmp(tokens[0], "S")) { //|| strcmp(tokens[0], "P") == 0) {
      if (allocated_samples <= num_samples) {
        samples = realloc(samples, ++allocated_samples * sizeof(coord));
      }
      
      double x = strtod(tokens[1], NULL);
      double y = strtod(tokens[2], NULL);
      
      //int path = strcmp(tokens[0], "P") == 0;
      //int path = 0;
      
      //double red = path ? 1 : strtod(tokens[3], NULL);
      //double green = path ? 1 : strtod(tokens[4], NULL);
      //double blue = path ? 1 : strtod(tokens[5], NULL);
      
      double red = strtod(tokens[3], NULL);
      double green = strtod(tokens[4], NULL);
      double blue = strtod(tokens[5], NULL);
      
      samples[num_samples].x = x;
      samples[num_samples].y = y;
      samples[num_samples].red = red;
      samples[num_samples].green = green;
      samples[num_samples].blue = blue;
      num_samples++;
    }*/
    // Reset sample point message.
    else if (strcmp(tokens[0], "R") == 0) {
      num_samples = 0;
    }
  }
}

void webots_physics_draw(int pass, const char *view) {
    int dataSize;
    const char *data = (const char *)dWebotsReceive(&dataSize);
    if (dataSize > 0) {
      char msg[dataSize];
      int count = 1, i = 0, j = 0;
      for ( ; i < dataSize; ++i) {
        char c = data[i];
        if (c == '\0') {
          msg[j] = c;
          process_message(msg);
          // reset for next string
          ++count;
          j = 0;
        } else {
          msg[j] = c;
          ++j;
        }
      }
    }
    
    if (pass == 1 && view == NULL) {
      // setup draw style
      glDisable(GL_LIGHTING);
      glLineWidth(1);
   
      // draw a yellow line at the goal
      if (received_goal) {
        glBegin(GL_LINES);
        glColor3f(1, 1, 0);
        glVertex3f(goal[0], 0, goal[1]);
        glVertex3f(goal[0], 3, goal[1]);
        glEnd();
      }
      
      /*glEnable(GL_POINT_SMOOTH);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glPointSize( 6.0 );
      
      glBegin(GL_POINTS);
      for ( int i = 0; i < num_samples; i++)
      {
        glColor3f( samples[i].red, samples[i].green, samples[1].blue);
        glVertex3f(samples[i].x, 0.1, samples[i].y);
      }
      glEnd();*/
    }
   /*
   * Note 1: 2 passes per rendering are performed, one before the 2D view rendering (pass = 0), and one after (pass = 1).
   *         You can handle that using the "pass" parameter.
   * Note 2: It is possible to draw in the robot cameras using the view parameter (it contains the Robot::name field)
   */
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  /*
   * This function needs to be implemented if you want to overide Webots collision detection.
   * It must return 1 if the collision was handled and 0 otherwise.
   * Note that contact joints should be added to the contact_joint_group which can change over the time, e.g.
   *   n = dCollide(g1, g2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
   *   dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();
   *   dWorldID world = dBodyGetWorld(body1);
   *   ...
   *   dJointCreateContact(world, contact_joint_group, &contact[i])
   *   dJointAttach(contact_joint, body1, body2);
   *   ...
   */
  return 0;
}

void webots_physics_cleanup() {
  free(samples);
}
