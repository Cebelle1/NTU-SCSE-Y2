**
 * @file server.c
 * This program creates a server that will listen for incoming connections.
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <semaphore.h>
#include <pthread.h>

// Declare global variables and stuff
#define BUFFER_SIZE 10
#define THREAD_COUNT 3

sem_t empty;
sem_t full;
pthread_mutex_t mutex;
int client_buffer[BUFFER_SIZE];
int in = 0;
int out = 0;

void error(char *error_message)
{
        perror(error_message);
        exit(0);
}

/**
 * @brief This function handles the client connection.
 *
 * @param arg
 * @return void*
 */
void *handle_client(void *arg)
{
        int newsockfd = *(int *)arg;
        char buffer[256];
        int n;

        // Connect, and start sending and reading message
        n = read(newsockfd, buffer, 255);
        if (n < 0)
        {
                error("Error reading from socket :/\n");
        }
	printf("Message received: %s\n", buffer);
        int number = atoi(buffer);

        memset(buffer, 0, sizeof(buffer));
        sprintf(buffer, "%d", number * 5);

        // Place client request in buffer
        sem_wait(&empty);
        pthread_mutex_lock(&mutex);
        client_buffer[in] = newsockfd;
        in = (in + 1) % BUFFER_SIZE;

        pthread_mutex_unlock(&mutex);
        sem_post(&full);

        sem_wait(&full);
        n = write(newsockfd, buffer, strlen(buffer));

        printf("Sent message: %s\n", buffer);

        if (n < 0)
        {
                error("Error writing to socket :/\n");
        }
        
        printf("Client disconnected\n");

        // close(newsockfd);

        sem_post(&empty);

        pthread_exit(NULL);
}

int main(int argc, char *argv[])
{
        // Initialise and stuff
        int sockfd, newsockfd, port, clilen;
        struct sockaddr_in serv_addr, cli_addr;

        if (argc < 2)
        {
                error("Usage: server [port]\n");
        }

        // Create socket and port, and initialise the server
        port = atoi(argv[1]);
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
        {
                error("Error opening socket :/\n");
        }

        // Clear server address structure for a clean start
        memset(((char *)&serv_addr), 0, sizeof(serv_addr));

        // Set address family, copy server address to server address structure, set port number
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(port);

        // Binding stuff
        if (bind(sockfd,(struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0)
        {
                error("Error binding to socket:/\n");
        }

        // Listen for incoming connections
        listen(sockfd, 2);
        clilen = sizeof(cli_addr);

// Initialise semaphore and mutex
        sem_init(&empty, 0, BUFFER_SIZE);
        sem_init(&full, 0, 0);
        pthread_mutex_init(&mutex, NULL);

        // Spawn threads to handle client connections
        pthread_t threads[THREAD_COUNT];
        int thread_args[THREAD_COUNT];

        // Connect here, don't need to reopen and close
        newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);

        if (newsockfd < 0)
        {
                error("Error accepting connection :/\n");
        }

        for (int i = 0; i < THREAD_COUNT; i++)
        {
                thread_args[i] = newsockfd;
                pthread_create(&threads[i], NULL, handle_client, (void *)&thread_args[i]);
        }
        
        for (int i = 0; i < THREAD_COUNT; i++)
        {
                pthread_join(threads[i], NULL);
                printf("Thread %d finished\n", i);
        }

        close(newsockfd);

        return 0;
}




