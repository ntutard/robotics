%{
#include <stdlib.h>
#include <stdio.h>
int indetationLvl;
void afficherIndentation();
%}
%%
"{" {printf("{\n");indetationLvl+=4;afficherIndentation();}
"}" {printf("}\n");indetationLvl-=4;afficherIndentation();}

%%
void afficherIndentation()
{
int i;
for(i=0;i< indetationLvl;i++)
	{
	printf(" ");
	}
	
}
