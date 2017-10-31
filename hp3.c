//Input format for this program:
//|Sq|
//|Si|
//Sq
//Si

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WINDOW_SIZE 3
#define MIN_SCORE 2
#define MATCH_SCORE 1
#define MISMATCH_SCORE 0


int max2(int a, int b)
{
	return (a>b)?a:b;
}

int max(int a, int b, int c)
{
	int temp = max2(a,b);
	return max2(temp,c);
}



int getAlignmentScore(char *query, char *target)
{
	//for our implementation, length of query and target are equal
	int length = strlen(query);
	int **table = (int **)malloc ((length+1)*(length+1)*sizeof(int));

	int i, j;
	for(i=0; i<=length; i++)
	{
		table[i] = (int *)malloc((length+1)*sizeof(int));
	}

	//base values for the table
	for(i=0; i<=length; i++)
	{
		table[i][0]=0;
		table[0][i]=0;
	}

	//DP implementation for string matching
	for(i=1; i<=length; i++)
	{
		for(j=1; j<=length; j++)
		{
			int s = (query[i-1] == target[j-1])?MATCH_SCORE:MISMATCH_SCORE;
			table[i][j] = max(table[i-1][j-1] + s, table[i][j-1], table[i-1][j]);
		}
	}

	return table[length][length];
}

//utility function which returns the matching score in a window of T
int getWindowScore(char *query, char *target)
{
	int i;
	int score=0;
	for(i=0; i<WINDOW_SIZE; i++)
	{
		score+=(query[i] == target[i])?MATCH_SCORE:MISMATCH_SCORE;
	}

	return score;
}

int main()
{
	int AlignmentScore=0;
	int Offset=-1;

	int sq_length;
	int si_length;
	printf("Enter the length of query string : ");
	scanf("%d", &sq_length);
	printf("Enter the length of target string : ");
	scanf("%d", &si_length);

	char *sq = (char *)malloc(sq_length);
	char *si = (char *)malloc(si_length);

	printf("Enter query string : ");
	scanf("%s", sq);
	printf("Enter target string : ");
	scanf("%s", si);


	int i, j;
	for(i=0; i<sq_length-WINDOW_SIZE+1; i++)
	{
		// char *query = sq+i;
		//for every such window,check for anchor point in Si
		for(j=0; j<si_length-sq_length+1; j++)
		{

			int window_score = getWindowScore(sq+i, si+i+j);
			// printf("window score %d %s %s\n",window_score, sq+i,si+i+j );
			if(window_score>=MIN_SCORE)
			{
				//this means it is an anchor point
				int align_score = getAlignmentScore(sq, si+j);
				AlignmentScore = max2(AlignmentScore, align_score);
				if(AlignmentScore == align_score)
					Offset=j;
			}
		}
	}

	char *aligned_string = (char *)malloc(sq_length+1);
	for(i=0; i<sq_length; i++)
	{
		aligned_string[i] = si[Offset+i];
	}

	aligned_string[sq_length]='\0';
	printf("Alignment Score = %d\n", AlignmentScore);
	printf("Query string\t: %s\n", sq);
	printf("Aligned String\t: %s\n", aligned_string);


	return 0;
}